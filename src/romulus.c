#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"
#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"
#include "lv2/lv2plug.in/ns/ext/midi/midi.h"
#include "lv2/lv2plug.in/ns/ext/time/time.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"

/* Pad size to 64-bit boundary */
#define lv2_atom_pad_size(size) (((size) + 7) & (~7))

#define PLUGIN_URI "http://github.com/lbovet/romulus"
#define MAX_MIDI_EVENTS 16384
#define MAX_ACTIVE_NOTES (16 * 128)  /* 16 channels * 128 notes */

/* Port indices */
typedef enum {
    PORT_MIDI_IN = 0,
    PORT_MIDI_OUT = 1,
    PORT_RECORD_ENABLE = 2,
    PORT_LOOP_LENGTH = 3
} PortIndex;

/* Plugin state */
typedef enum {
    STATE_IDLE,           /* Not recording, not playing */
    STATE_ARMED,          /* Waiting for next bar to start recording */
    STATE_RECORDING,      /* Currently recording */
    STATE_PLAYING         /* Playing back the loop */
} PluginState;

/* MIDI event structure */
typedef struct {
    uint32_t frame;       /* Frame offset within loop */
    uint32_t size;        /* Size of MIDI message */
    uint8_t msg[3];       /* MIDI message data */
    bool ignored;         /* Skip this event during playback */
} MidiEvent;

/* Active note tracking */
typedef struct {
    bool active;          /* Is this note currently on? */
    uint8_t channel;      /* MIDI channel */
    uint8_t note;         /* Note number */
    bool waiting_noteoff; /* Waiting for note-off after loop end */
} ActiveNote;

/* URIDs for Atom communication */
typedef struct {
    LV2_URID atom_Blank;
    LV2_URID atom_Object;
    LV2_URID atom_Sequence;
    LV2_URID atom_Float;
    LV2_URID atom_Long;
    LV2_URID midi_MidiEvent;
    LV2_URID time_Position;
    LV2_URID time_bar;
    LV2_URID time_barBeat;
    LV2_URID time_beatsPerMinute;
    LV2_URID time_beatsPerBar;
    LV2_URID time_speed;
} URIDs;

/* Plugin instance */
typedef struct {
    /* Ports */
    const LV2_Atom_Sequence* midi_in;
    LV2_Atom_Sequence* midi_out;
    const float* record_enable;
    const float* loop_length;
    
    /* URIDs */
    URIDs urids;
    LV2_URID_Map* map;
    
    /* State */
    PluginState state;
    float prev_record_enable;
    
    /* Transport info */
    double sample_rate;
    double bpm;
    double beats_per_bar;
    double current_bar;
    double current_beat;
    bool transport_rolling;
    
    /* Loop info */
    uint32_t loop_start_frame;  /* Absolute frame when loop recording started */
    uint32_t loop_length_frames; /* Length of loop in frames */
    uint32_t current_loop_frame; /* Current position in loop playback */
    
    /* MIDI events storage */
    MidiEvent events[MAX_MIDI_EVENTS];
    uint32_t event_count;
    
    /* Active notes tracking */
    ActiveNote active_notes[MAX_ACTIVE_NOTES];
    
} Romulus;

/* Map URIDs */
static void
map_urids(LV2_URID_Map* map, URIDs* urids)
{
    urids->atom_Blank = map->map(map->handle, LV2_ATOM__Blank);
    urids->atom_Object = map->map(map->handle, LV2_ATOM__Object);
    urids->atom_Sequence = map->map(map->handle, LV2_ATOM__Sequence);
    urids->atom_Float = map->map(map->handle, LV2_ATOM__Float);
    urids->atom_Long = map->map(map->handle, LV2_ATOM__Long);
    urids->midi_MidiEvent = map->map(map->handle, LV2_MIDI__MidiEvent);
    urids->time_Position = map->map(map->handle, LV2_TIME__Position);
    urids->time_bar = map->map(map->handle, LV2_TIME__bar);
    urids->time_barBeat = map->map(map->handle, LV2_TIME__barBeat);
    urids->time_beatsPerMinute = map->map(map->handle, LV2_TIME__beatsPerMinute);
    urids->time_beatsPerBar = map->map(map->handle, LV2_TIME__beatsPerBar);
    urids->time_speed = map->map(map->handle, LV2_TIME__speed);
}

/* Instantiate the plugin */
static LV2_Handle
instantiate(const LV2_Descriptor* descriptor,
           double rate,
           const char* bundle_path,
           const LV2_Feature* const* features)
{
    Romulus* self = (Romulus*)calloc(1, sizeof(Romulus));
    if (!self) {
        return NULL;
    }
    
    /* Get the URID map feature */
    for (int i = 0; features[i]; ++i) {
        if (!strcmp(features[i]->URI, LV2_URID__map)) {
            self->map = (LV2_URID_Map*)features[i]->data;
        }
    }
    
    if (!self->map) {
        fprintf(stderr, "Romulus: Host does not support urid:map\n");
        free(self);
        return NULL;
    }
    
    map_urids(self->map, &self->urids);
    
    fprintf(stderr, "Romulus: Plugin instantiated at sample rate %.0f\n", rate);
    fprintf(stderr, "Romulus: URIDs - midi=%u, pos=%u, float=%u, long=%u\n",
           self->urids.midi_MidiEvent, self->urids.time_Position,
           self->urids.atom_Float, self->urids.atom_Long);
    
    self->sample_rate = rate;
    self->state = STATE_IDLE;
    self->prev_record_enable = 1.0f;
    self->bpm = 120.0;
    self->beats_per_bar = 4.0;
    self->transport_rolling = false;
    self->event_count = 0;
    
    /* Initialize active notes */
    for (int i = 0; i < MAX_ACTIVE_NOTES; ++i) {
        self->active_notes[i].active = false;
        self->active_notes[i].waiting_noteoff = false;
    }
    
    return (LV2_Handle)self;
}

/* Connect ports */
static void
connect_port(LV2_Handle instance, uint32_t port, void* data)
{
    Romulus* self = (Romulus*)instance;
    
    switch ((PortIndex)port) {
    case PORT_MIDI_IN:
        self->midi_in = (const LV2_Atom_Sequence*)data;
        break;
    case PORT_MIDI_OUT:
        self->midi_out = (LV2_Atom_Sequence*)data;
        break;
    case PORT_RECORD_ENABLE:
        self->record_enable = (const float*)data;
        break;
    case PORT_LOOP_LENGTH:
        self->loop_length = (const float*)data;
        break;
    }
}

/* Activate the plugin */
static void
activate(LV2_Handle instance)
{
    /* Nothing to do here */
}

/* Add a MIDI event to the loop */
static void
add_event(Romulus* self, uint32_t frame, const uint8_t* msg, uint32_t size)
{
    if (self->event_count >= MAX_MIDI_EVENTS) {
        return; /* Buffer full */
    }
    
    MidiEvent* event = &self->events[self->event_count++];
    event->frame = frame;
    event->size = size;
    event->ignored = false;
    memcpy(event->msg, msg, size < 3 ? size : 3);
}

/* Clear all events from the loop */
static void
clear_events(Romulus* self)
{
    self->event_count = 0;
}

/* Get note index for tracking (channel * 128 + note) */
static inline int
get_note_index(uint8_t channel, uint8_t note)
{
    return (channel & 0x0F) * 128 + (note & 0x7F);
}

/* Mark a note as active */
static void
note_on(Romulus* self, uint8_t channel, uint8_t note)
{
    int idx = get_note_index(channel, note);
    self->active_notes[idx].active = true;
    self->active_notes[idx].channel = channel;
    self->active_notes[idx].note = note;
}

/* Mark a note as inactive */
static void
note_off(Romulus* self, uint8_t channel, uint8_t note)
{
    int idx = get_note_index(channel, note);
    self->active_notes[idx].active = false;
    
    /* If we were waiting for this note-off, stop waiting */
    if (self->active_notes[idx].waiting_noteoff) {
        self->active_notes[idx].waiting_noteoff = false;
    }
}

/* Start waiting for note-offs of currently active notes */
static void
start_waiting_for_noteoffs(Romulus* self)
{
    int waiting_count = 0;
    for (int i = 0; i < MAX_ACTIVE_NOTES; ++i) {
        if (self->active_notes[i].active) {
            self->active_notes[i].waiting_noteoff = true;
            waiting_count++;
        }
    }
    fprintf(stderr, "Romulus: Waiting for %d note-offs\n", waiting_count);
}

/* Calculate frames per beat */
static inline double
frames_per_beat(Romulus* self)
{
    return (self->sample_rate * 60.0) / self->bpm;
}

/* Update transport information from position */
static void
update_transport(Romulus* self, const LV2_Atom_Object* obj)
{
    LV2_Atom* bar = NULL;
    LV2_Atom* barBeat = NULL;
    LV2_Atom* bpm_atom = NULL;
    LV2_Atom* bpb = NULL;
    LV2_Atom* speed = NULL;
    
    static int log_count = 0;
    if (log_count < 5) {
        fprintf(stderr, "Romulus: Received time:Position event\n");
        log_count++;
    }
    
    lv2_atom_object_get(obj,
                       self->urids.time_bar, &bar,
                       self->urids.time_barBeat, &barBeat,
                       self->urids.time_beatsPerMinute, &bpm_atom,
                       self->urids.time_beatsPerBar, &bpb,
                       self->urids.time_speed, &speed,
                       NULL);
    
    if (bar && bar->type == self->urids.atom_Long) {
        self->current_bar = (double)((LV2_Atom_Long*)bar)->body;
    }
    
    if (barBeat && barBeat->type == self->urids.atom_Float) {
        self->current_beat = (double)((LV2_Atom_Float*)barBeat)->body;
    }
    
    if (bpm_atom && bpm_atom->type == self->urids.atom_Float) {
        self->bpm = (double)((LV2_Atom_Float*)bpm_atom)->body;
    }
    
    if (bpb && bpb->type == self->urids.atom_Float) {
        self->beats_per_bar = (double)((LV2_Atom_Float*)bpb)->body;
    }
    
    if (speed && speed->type == self->urids.atom_Float) {
        float speed_val = ((LV2_Atom_Float*)speed)->body;
        bool was_rolling = self->transport_rolling;
        self->transport_rolling = (speed_val > 0.0f);
        if (!was_rolling && self->transport_rolling) {
            fprintf(stderr, "Romulus: Transport started (bar=%.1f, beat=%.3f, bpm=%.1f, speed=%.2f)\n", 
                   self->current_bar, self->current_beat, self->bpm, speed_val);
        }
    } else if (log_count < 5) {
        fprintf(stderr, "Romulus: No speed info in Position (speed=%p, type=%u vs %u)\n",
               (void*)speed, speed ? speed->type : 0, self->urids.atom_Float);
    }
}

/* Check if we're at the start of a bar */
static bool
is_bar_start(Romulus* self)
{
    /* Consider it a bar start if we're very close to beat 0 */
    return self->current_beat < 0.01;
}

/* Run the plugin */
static void
run(LV2_Handle instance, uint32_t n_samples)
{
    Romulus* self = (Romulus*)instance;
    
    /* Get the capacity of the output port */
    const uint32_t out_capacity = self->midi_out->atom.size;
    
    /* Initialize output sequence */
    lv2_atom_sequence_clear(self->midi_out);
    self->midi_out->atom.type = self->urids.atom_Sequence;
    self->midi_out->body.unit = 0;  /* Frame time */
    self->midi_out->body.pad = 0;
    
    /* Read control ports */
    float record_enable = *self->record_enable;
    float loop_length_bars = *self->loop_length;
    
    /* Detect record enable toggle (from 1 to 0) */
    bool arm_recording = false;
    if (self->prev_record_enable >= 0.5f && record_enable < 0.5f) {
        arm_recording = true;
    }
    self->prev_record_enable = record_enable;
    
    static int run_count = 0;
    if (run_count < 3) {
        fprintf(stderr, "Romulus: run() called, n_samples=%u, state=%d\n", n_samples, self->state);
        run_count++;
    }
    
    static int event_log_count = 0;
    int event_count_this_run = 0;
    
    /* Process incoming MIDI events */
    LV2_ATOM_SEQUENCE_FOREACH(self->midi_in, ev) {
        event_count_this_run++;
        if (event_log_count < 10) {
            fprintf(stderr, "Romulus: Event #%d type=%u size=%u (midi=%u, pos=%u, obj=%u, blank=%u)\n",
                   event_count_this_run, ev->body.type, ev->body.size,
                   self->urids.midi_MidiEvent, self->urids.time_Position,
                   self->urids.atom_Object, self->urids.atom_Blank);
            event_log_count++;
        }
        if (ev->body.type == self->urids.midi_MidiEvent) {
            const uint8_t* msg = (const uint8_t*)(ev + 1);
            uint8_t status = msg[0] & 0xF0;
            uint8_t channel = msg[0] & 0x0F;
            
            /* Track note on/off */
            if (status == 0x90 && msg[2] > 0) { /* Note on */
                note_on(self, channel, msg[1]);
            } else if (status == 0x80 || (status == 0x90 && msg[2] == 0)) { /* Note off */
                note_off(self, channel, msg[1]);
            }
            
            /* Handle based on state */
            if (self->state == STATE_RECORDING) {
                /* Add event to loop */
                uint32_t loop_frame = self->loop_start_frame + ev->time.frames;
                fprintf(stderr, "Romulus: MIDI event during recording - ev->time.frames=%lu, loop_start_frame=%u, loop_frame=%u, loop_length=%u\n",
                       (unsigned long)ev->time.frames, self->loop_start_frame, loop_frame, self->loop_length_frames);
                if (loop_frame < self->loop_length_frames) {
                    add_event(self, loop_frame, msg, ev->body.size);
                    fprintf(stderr, "Romulus: Recorded event #%u (status=0x%02x, frame=%u)\n", 
                           self->event_count, msg[0], loop_frame);
                } else {
                    fprintf(stderr, "Romulus: Event skipped - loop_frame (%u) >= loop_length (%u)\n",
                           loop_frame, self->loop_length_frames);
                }
            } else if (self->state == STATE_PLAYING) {
                /* Record note-off if we're waiting for it */
                bool is_noteoff = (status == 0x80) || (status == 0x90 && msg[2] == 0);
                if (is_noteoff) {
                    int idx = get_note_index(channel, msg[1]);
                    if (self->active_notes[idx].waiting_noteoff) {
                        /* Add the note-off to the loop */
                        fprintf(stderr, "Romulus: Received waited note-off (ch=%d, note=%d)\n", channel, msg[1]);
                        add_event(self, self->loop_length_frames - 1, msg, ev->body.size);
                    }
                }
            }
        } else if (ev->body.type == self->urids.time_Position ||
                   ev->body.type == self->urids.atom_Object ||
                   ev->body.type == self->urids.atom_Blank) {
            /* Update transport information */
            if (event_log_count < 10) {
                fprintf(stderr, "Romulus: Processing position/object event (type=%u)\n", ev->body.type);
            }
            update_transport(self, (const LV2_Atom_Object*)&ev->body);
        }
    }
    
    /* Handle arming recording */
    if (arm_recording) {
        fprintf(stderr, "Romulus: Record armed (current state: %d)\n", self->state);
        if (self->state == STATE_RECORDING) {
            /* Discard current recording and rearm */
            clear_events(self);
            for (int i = 0; i < MAX_ACTIVE_NOTES; ++i) {
                self->active_notes[i].active = false;
                self->active_notes[i].waiting_noteoff = false;
            }
            self->state = STATE_ARMED;
        } else {
            self->state = STATE_ARMED;
        }
    }
    
    /* State machine */
    if (self->state == STATE_ARMED && self->transport_rolling) {
        /* Check if we've reached a bar boundary */
        if (is_bar_start(self)) {
            /* Start recording */
            fprintf(stderr, "Romulus: Starting recording (loop_length_bars=%.0f, bpm=%.1f, beats_per_bar=%.1f)\n", 
                   loop_length_bars, self->bpm, self->beats_per_bar);
            self->state = STATE_RECORDING;
            self->loop_start_frame = 0; /* Will be set relative to current frame */
            double fpb = frames_per_beat(self);
            self->loop_length_frames = (uint32_t)(fpb * self->beats_per_bar * loop_length_bars);
            clear_events(self);
            
            /* Clear active notes tracking */
            for (int i = 0; i < MAX_ACTIVE_NOTES; ++i) {
                self->active_notes[i].active = false;
                self->active_notes[i].waiting_noteoff = false;
            }
        }
    } else if (self->state == STATE_RECORDING) {
        /* Check if loop length is reached */
        /* Note: This is simplified - a real implementation would track exact frame counts */
        if (self->loop_start_frame >= self->loop_length_frames) {
            /* Recording complete - start playback immediately */
            fprintf(stderr, "Romulus: Recording complete, %u events recorded\n", self->event_count);
            start_waiting_for_noteoffs(self);
            self->state = STATE_PLAYING;
            self->current_loop_frame = 0;
            fprintf(stderr, "Romulus: Starting playback\n");
        }
        self->loop_start_frame += n_samples;
    } else if (self->state == STATE_PLAYING && self->transport_rolling) {
        /* Play back loop aligned to bars */
        if (is_bar_start(self)) {
            self->current_loop_frame = 0;
        }
        
        /* Output events that fall within this buffer */
        for (uint32_t i = 0; i < self->event_count; ++i) {
            MidiEvent* event = &self->events[i];
            
            /* event->frame is the absolute position in the loop */
            if (event->frame >= self->current_loop_frame && 
                event->frame < self->current_loop_frame + n_samples) {
                
                uint32_t offset = event->frame - self->current_loop_frame;
                if (offset < n_samples) {
                    fprintf(stderr, "Romulus: Playing event #%u at loop_frame=%u, offset=%u\n",
                           i+1, event->frame, offset);
                    /* Check event type */
                    const uint8_t* msg = event->msg;
                    uint8_t status = msg[0] & 0xF0;
                    uint8_t channel = msg[0] & 0x0F;
                    bool is_noteoff = (status == 0x80) || (status == 0x90 && msg[2] == 0);
                    bool is_noteon = (status == 0x90 && msg[2] > 0);
                    
                    int idx = get_note_index(channel, msg[1]);
                    
                    if (is_noteoff && self->active_notes[idx].waiting_noteoff) {
                        /* Mark this note-off event as ignored and skip it */
                        event->ignored = true;
                    } else if (is_noteon && self->active_notes[idx].waiting_noteoff) {
                        /* Note-on from loop while waiting - stop waiting for note-off */
                        self->active_notes[idx].waiting_noteoff = false;
                    }
                    
                    /* Skip ignored events */
                    if (!event->ignored) {
                        fprintf(stderr, "Romulus: Writing MIDI to output: [%02x %02x %02x] size=%u, offset=%u\n",
                               event->msg[0], event->msg[1], event->msg[2], event->size, offset);
                        
                        /* Calculate padded event size */
                        uint32_t padded_size = lv2_atom_pad_size(sizeof(LV2_Atom_Event) + event->size);
                        
                        /* Check if there's enough space */
                        if (self->midi_out->atom.size + padded_size > out_capacity) {
                            fprintf(stderr, "Romulus: Output buffer full!\n");
                            break;
                        }
                        
                        /* Get pointer to next event location */
                        LV2_Atom_Event* out_ev = (LV2_Atom_Event*)
                            ((uint8_t*)LV2_ATOM_CONTENTS(LV2_Atom_Sequence, self->midi_out) +
                             self->midi_out->atom.size);
                        
                        /* Write event header */
                        out_ev->time.frames = offset;
                        out_ev->body.type = self->urids.midi_MidiEvent;
                        out_ev->body.size = event->size;
                        
                        /* Write MIDI data */
                        memcpy(out_ev + 1, event->msg, event->size);
                        
                        /* Update sequence size with padding */
                        self->midi_out->atom.size += padded_size;
                    }
                }
            }
        }
        
        self->current_loop_frame += n_samples;
        if (self->current_loop_frame >= self->loop_length_frames) {
            self->current_loop_frame = 0;
        }
    }
}

/* Deactivate the plugin */
static void
deactivate(LV2_Handle instance)
{
    /* Nothing to do here */
}

/* Cleanup and deallocate */
static void
cleanup(LV2_Handle instance)
{
    free(instance);
}

/* Extension data */
static const void*
extension_data(const char* uri)
{
    return NULL;
}

/* Plugin descriptor */
static const LV2_Descriptor descriptor = {
    PLUGIN_URI,
    instantiate,
    connect_port,
    activate,
    run,
    deactivate,
    cleanup,
    extension_data
};

/* Entry point */
LV2_SYMBOL_EXPORT
const LV2_Descriptor*
lv2_descriptor(uint32_t index)
{
    return index == 0 ? &descriptor : NULL;
}
