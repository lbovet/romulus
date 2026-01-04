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
#include "lv2/lv2plug.in/ns/ext/state/state.h"
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
    PORT_LOOP_LENGTH = 3,
    PORT_PERSIST_LOOP = 4,
    PORT_ARMED = 5,
    PORT_RECORDING = 6,
    PORT_RECORDED = 7,
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
    LV2_URID atom_frameTime;
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
    float* armed;
    float* recording;
    float* recorded;
    const float* persist_loop;
    
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
    int64_t transport_frame;      /* Current frame position from host */
    int64_t bar_start_frame;      /* Frame position of the most recent bar start */
    bool transport_rolling;
    bool transport_just_stopped;
    uint32_t transport_update_count;
    
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
    urids->atom_frameTime = map->map(map->handle, "http://lv2plug.in/ns/ext/atom#frameTime");
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
    
    self->sample_rate = rate;
    self->state = STATE_IDLE;
    self->prev_record_enable = 0.0f;
    self->bpm = 120.0;
    self->beats_per_bar = 4.0;
    self->transport_frame = 0;
    self->bar_start_frame = 0;
    self->transport_rolling = false;
    self->transport_just_stopped = false;
    self->transport_update_count = 0;
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
    case PORT_PERSIST_LOOP:
        self->persist_loop = (const float*)data;
        break;        
    case PORT_ARMED:
        self->armed = (float*)data;
        break;
    case PORT_RECORDING:
        self->recording = (float*)data;
        break;
    case PORT_RECORDED:
        self->recorded = (float*)data;
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
}

/* Start waiting for note-offs of currently active notes */
static void
start_waiting_for_noteoffs(Romulus* self)
{
    int waiting_count = 0;
    for (int i = 0; i < MAX_ACTIVE_NOTES; ++i) {
        if (self->active_notes[i].active) {
            fprintf(stderr, "Romulus: Waiting for note-off ch=%d note=%d\n",
                   self->active_notes[i].channel, self->active_notes[i].note);
            self->active_notes[i].waiting_noteoff = true;
            waiting_count++;
        }
    }
    fprintf(stderr, "Romulus: Waiting for %d note-offs total\n", waiting_count);
}

/* Send note-off for all note-on events in the loop */
static void
send_all_notes_off(Romulus* self, uint32_t out_capacity)
{
    /* Iterate through all recorded events and send note-offs for note-ons */
    for (uint32_t i = 0; i < self->event_count; ++i) {
        MidiEvent* event = &self->events[i];
        uint8_t status = event->msg[0] & 0xF0;
        
        /* Check if it's a note-on */
        if (status == 0x90 && event->msg[2] > 0) {
            /* Create a note-off message */
            uint8_t noteoff_msg[3];
            noteoff_msg[0] = 0x80 | (event->msg[0] & 0x0F); /* Note-off with same channel */
            noteoff_msg[1] = event->msg[1]; /* Same note */
            noteoff_msg[2] = 0x40; /* Velocity 64 */
            
            /* Calculate padded event size */
            uint32_t padded_size = lv2_atom_pad_size(sizeof(LV2_Atom_Event) + 3);
            
            /* Check if there's enough space */
            if (self->midi_out->atom.size + padded_size > out_capacity) {
                break;
            }
            
            /* Get pointer to next event location */
            LV2_Atom_Event* out_ev = (LV2_Atom_Event*)
                ((uint8_t*)LV2_ATOM_CONTENTS(LV2_Atom_Sequence, self->midi_out) +
                 self->midi_out->atom.size - sizeof(LV2_Atom_Sequence_Body));
            
            /* Write event header */
            out_ev->time.frames = 0; /* Send immediately */
            out_ev->body.type = self->urids.midi_MidiEvent;
            out_ev->body.size = 3;
            
            /* Write MIDI data */
            uint8_t* midi_dest = (uint8_t*)(out_ev + 1);
            memcpy(midi_dest, noteoff_msg, 3);
            
            /* Update sequence size */
            self->midi_out->atom.size += padded_size;
        }
    }
}

/* Calculate frames per beat */
static inline double
frames_per_beat(Romulus* self)
{
    return (self->sample_rate * 60.0) / self->bpm;
}

/* Update transport information from position */
static void
update_transport(Romulus* self, const LV2_Atom_Object* obj, int64_t frame_offset)
{
    LV2_Atom* bar = NULL;
    LV2_Atom* barBeat = NULL;
    LV2_Atom* bpm_atom = NULL;
    LV2_Atom* bpb = NULL;
    LV2_Atom* speed = NULL;
    LV2_Atom* frame_atom = NULL;
    
    /* Get frame position */
    LV2_URID frame_urid = self->map->map(self->map->handle, LV2_TIME__frame);
    
    lv2_atom_object_get(obj,
                       frame_urid, &frame_atom,
                       self->urids.time_bar, &bar,
                       self->urids.time_barBeat, &barBeat,
                       self->urids.time_beatsPerMinute, &bpm_atom,
                       self->urids.time_beatsPerBar, &bpb,
                       self->urids.time_speed, &speed,
                       NULL);
    
    /* Update frame position */
    if (frame_atom && frame_atom->type == self->urids.atom_Long) {
        self->transport_frame = ((LV2_Atom_Long*)frame_atom)->body + frame_offset;
    }
    
    /* If we get bar/barBeat, calculate the frame position of the bar start */
    if (bar && bar->type == self->urids.atom_Long &&
        barBeat && barBeat->type == self->urids.atom_Float) {
        
        double beat_in_bar = (double)((LV2_Atom_Float*)barBeat)->body;
        double frames_per_beat_val = frames_per_beat(self);
        int64_t frames_from_bar_start = (int64_t)(beat_in_bar * frames_per_beat_val);
        self->bar_start_frame = self->transport_frame - frames_from_bar_start;
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
        
        /* Detect transport stop */
        if (was_rolling && !self->transport_rolling) {
            self->transport_just_stopped = true;
            /* If we were playing, go back to IDLE */
            if (self->state == STATE_PLAYING) {
                fprintf(stderr, "Romulus: Transport stopped, PLAYING -> IDLE\n");
                self->state = STATE_IDLE;
            }
        }
        
        /* Detect transport start - schedule playing */
        if (!was_rolling && self->transport_rolling) {
            fprintf(stderr, "Romulus: Transport started -> IDLE\n");
            self->state = STATE_IDLE;
        }
    }
    
}

/* Check if we're at the start of a bar based on frame position */
static bool
is_bar_start(Romulus* self, int64_t current_frame)
{
    /* Calculate frames per bar */
    double frames_per_beat_val = frames_per_beat(self);
    int64_t frames_per_bar = (int64_t)(frames_per_beat_val * self->beats_per_bar);
    
    /* Calculate position within the current bar */
    int64_t frames_since_bar_start = current_frame - self->bar_start_frame;
    int64_t position_in_bar = frames_since_bar_start % frames_per_bar;
    
    /* We're at bar start if position is within first 256 frames (about 5ms at 48kHz) */
    return position_in_bar < 256;
}

/* Run the plugin */
static void
run(LV2_Handle instance, uint32_t n_samples)
{
    Romulus* self = (Romulus*)instance;
    
    /* Get capacity before we touch the buffer */
    const uint32_t out_capacity = self->midi_out->atom.size;
    
    /* Always initialize the sequence properly - host has cleared it */
    self->midi_out->atom.type = self->urids.atom_Sequence;
    self->midi_out->atom.size = sizeof(LV2_Atom_Sequence_Body);  
    self->midi_out->body.unit = self->urids.atom_frameTime;
    self->midi_out->body.pad = 0;
    
    /* Track frame position at start of buffer */
    int64_t buffer_start_frame = self->transport_frame;
    
    /* Read control ports */
    float record_enable = *self->record_enable;
    float loop_length_bars = *self->loop_length;
    
    /* Send all notes off if transport just stopped */
    if (self->transport_just_stopped && self->event_count > 0) {
        send_all_notes_off(self, out_capacity);
        self->transport_just_stopped = false;
    }
    
    /* Detect record enable toggle (from 1 to 0) */
    bool arm_recording = false;
    if (self->prev_record_enable >= 0.5f && record_enable < 0.5f) {
        arm_recording = true;
    }
    self->prev_record_enable = record_enable;
    
    /* Process incoming MIDI events */
    LV2_ATOM_SEQUENCE_FOREACH(self->midi_in, ev) {
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
                if (loop_frame < self->loop_length_frames) {
                    add_event(self, loop_frame, msg, ev->body.size);
                }
            } else if (self->state == STATE_PLAYING) {
                /* Record note-off if we're waiting for it */
                bool is_noteoff = (status == 0x80) || (status == 0x90 && msg[2] == 0);
                if (is_noteoff) {
                    int idx = get_note_index(channel, msg[1]);
                    if (self->active_notes[idx].waiting_noteoff) {
                        /* Calculate the position in the loop where this note-off occurred */
                        uint32_t noteoff_frame = (self->current_loop_frame + ev->time.frames) % self->loop_length_frames;
                        add_event(self, noteoff_frame, msg, ev->body.size);
                        
                        /* Stop waiting for this note-off */
                        fprintf(stderr, "Romulus: Received awaited note-off ch=%d note=%d\n",
                               channel, msg[1]);
                        self->active_notes[idx].waiting_noteoff = false;
                    }
                }
            }
        } else if (ev->body.type == self->urids.time_Position ||
                   ev->body.type == self->urids.atom_Object ||
                   ev->body.type == self->urids.atom_Blank) {
            /* Update transport information */
            update_transport(self, (const LV2_Atom_Object*)&ev->body, ev->time.frames);
        }
    }
    
    /* Handle arming recording */
    if (arm_recording) {
        /* Send all notes off before arming */
        if (self->event_count > 0) {
            send_all_notes_off(self, out_capacity);
        }
        
        if (self->state == STATE_RECORDING) {
            /* Discard current recording and rearm */
            clear_events(self);
            for (int i = 0; i < MAX_ACTIVE_NOTES; ++i) {
                self->active_notes[i].active = false;
                self->active_notes[i].waiting_noteoff = false;
            }
            fprintf(stderr, "Romulus: RECORDING -> ARMED (discarding loop)\n");
            self->state = STATE_ARMED;
        } else {
            fprintf(stderr, "Romulus: IDLE -> ARMED\n");
            self->state = STATE_ARMED;
        }
    }
    
    /* State machine */
    if (self->state == STATE_IDLE && self->transport_rolling && self->event_count > 0) {
        /* Check if we've reached a bar boundary */
        if (is_bar_start(self, buffer_start_frame)) {
            fprintf(stderr, "Romulus: IDLE -> PLAYING at bar start (%u events)\n", 
                   self->event_count);
            self->state = STATE_PLAYING;
            self->current_loop_frame = 0;
        }
    } else if (self->state == STATE_ARMED && self->transport_rolling) {
        /* Check if we've reached a bar boundary */
        if (is_bar_start(self, buffer_start_frame)) {
            /* Start recording */
            double fpb = frames_per_beat(self);
            uint32_t loop_frames = (uint32_t)(fpb * self->beats_per_bar * loop_length_bars);
            fprintf(stderr, "Romulus: ARMED -> RECORDING (%.0f bars, %u frames)\n", 
                   loop_length_bars, loop_frames);
            self->state = STATE_RECORDING;
            self->loop_start_frame = 0; /* Will be set relative to current frame */
            self->loop_length_frames = loop_frames;
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
            start_waiting_for_noteoffs(self);
            fprintf(stderr, "Romulus: RECORDING -> PLAYING (%u events)\n", self->event_count);
            self->state = STATE_PLAYING;
            self->current_loop_frame = 0;
        }
        self->loop_start_frame += n_samples;
    } else if (self->state == STATE_PLAYING && self->transport_rolling) {
        /* Output events that fall within this buffer */
        for (uint32_t i = 0; i < self->event_count; ++i) {
            MidiEvent* event = &self->events[i];
            
            /* event->frame is the absolute position in the loop */
            if (event->frame >= self->current_loop_frame && 
                event->frame < self->current_loop_frame + n_samples) {
                
                uint32_t offset = event->frame - self->current_loop_frame;
                
                if (offset < n_samples) {
                    /* Check event type */
                    const uint8_t* msg = event->msg;
                    uint8_t status = msg[0] & 0xF0;
                    uint8_t channel = msg[0] & 0x0F;
                    bool is_noteoff = (status == 0x80) || (status == 0x90 && msg[2] == 0);
                    bool is_noteon = (status == 0x90 && msg[2] > 0);
                    
                    int idx = get_note_index(channel, msg[1]);
                    
                    if (is_noteoff && self->active_notes[idx].waiting_noteoff) {
                        /* Mark this note-off event as ignored and skip it */
                        fprintf(stderr, "Romulus: Ignoring note-off ch=%d note=%d (waiting for external note-off)\n",
                               channel, msg[1]);
                        event->ignored = true;
                    } else if (is_noteon && self->active_notes[idx].waiting_noteoff) {
                        /* Note-on from loop while waiting - stop waiting for note-off */
                        fprintf(stderr, "Romulus: Note-on ch=%d note=%d while waiting - stopping wait\n",
                               channel, msg[1]);
                        self->active_notes[idx].waiting_noteoff = false;
                    }
                    
                    /* Skip ignored events */
                    if (!event->ignored) {
                        /* Calculate padded event size */
                        uint32_t padded_size = lv2_atom_pad_size(sizeof(LV2_Atom_Event) + event->size);
                        
                        /* Check if there's enough space */
                        if (self->midi_out->atom.size + padded_size > out_capacity) {
                            break;
                        }
                        
                        /* Get pointer to next event location */
                        LV2_Atom_Event* out_ev = (LV2_Atom_Event*)
                            ((uint8_t*)LV2_ATOM_CONTENTS(LV2_Atom_Sequence, self->midi_out) +
                             self->midi_out->atom.size - sizeof(LV2_Atom_Sequence_Body));
                        
                        /* Write event header */
                        out_ev->time.frames = offset;
                        out_ev->body.type = self->urids.midi_MidiEvent;
                        out_ev->body.size = event->size;
                        
                        /* Write MIDI data */
                        uint8_t* midi_dest = (uint8_t*)(out_ev + 1);
                        memcpy(midi_dest, event->msg, event->size);
                        
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
    
    /* Update status output ports */
    *self->armed = (self->state == STATE_ARMED) ? 1.0f : 0.0f;
    *self->recording = (self->state == STATE_RECORDING) ? 1.0f : 0.0f;
    *self->recorded = (self->state == STATE_PLAYING) ? 1.0f : 0.0f;
    
    /* Update frame position for next cycle */
    self->transport_frame += n_samples;
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

/* Save state */
static LV2_State_Status
save(LV2_Handle                instance,
     LV2_State_Store_Function  store,
     LV2_State_Handle          handle,
     uint32_t                  flags,
     const LV2_Feature* const* features)
{
    Romulus* self = (Romulus*)instance;
    
    fprintf(stderr, "Romulus: Saving requested\n");

    /* Only save if persist_loop is enabled */
    if (*self->persist_loop < 0.5f) {
        fprintf(stderr, "Romulus: Not saving state because persist_loop is disabled\n");
        return LV2_STATE_SUCCESS;
    }
    
    (void)features;  /* Unused */

    LV2_URID event_count_urid = self->map->map(self->map->handle, PLUGIN_URI "#event_count");
    LV2_URID loop_length_frames_urid = self->map->map(self->map->handle, PLUGIN_URI "#loop_length_frames");
    LV2_URID events_urid = self->map->map(self->map->handle, PLUGIN_URI "#events");
    
    /* Save event count */
    store(handle, event_count_urid,
          &self->event_count, sizeof(uint32_t),
          self->urids.atom_Long,
          LV2_STATE_IS_POD | LV2_STATE_IS_PORTABLE);
    
    /* Save loop length */
    store(handle, loop_length_frames_urid,
          &self->loop_length_frames, sizeof(uint32_t),
          self->urids.atom_Long,
          LV2_STATE_IS_POD | LV2_STATE_IS_PORTABLE);
    
    /* Save events array */
    if (self->event_count > 0) {
        fprintf(stderr, "Romulus: Saving %u events, size=%lu bytes\n",
               self->event_count, sizeof(MidiEvent) * self->event_count);
        store(handle, events_urid,
              self->events, sizeof(MidiEvent) * self->event_count,
              self->urids.atom_Long,  /* Use Long for binary data blob */
              LV2_STATE_IS_POD | LV2_STATE_IS_PORTABLE);
    }
    
    return LV2_STATE_SUCCESS;
}

/* Restore state */
static LV2_State_Status
restore(LV2_Handle                  instance,
        LV2_State_Retrieve_Function retrieve,
        LV2_State_Handle            handle,
        uint32_t                    flags,
        const LV2_Feature* const*   features)
{
    Romulus* self = (Romulus*)instance;
    
    (void)features;  /* Unused */
    
    fprintf(stderr, "Romulus: Restore requested\n");

    LV2_URID event_count_urid = self->map->map(self->map->handle, PLUGIN_URI "#event_count");
    LV2_URID loop_length_frames_urid = self->map->map(self->map->handle, PLUGIN_URI "#loop_length_frames");
    LV2_URID events_urid = self->map->map(self->map->handle, PLUGIN_URI "#events");
    
    /* Restore event count */
    size_t size;
    uint32_t type;
    uint32_t valflags;
    
    const void* value = retrieve(handle, event_count_urid, &size, &type, &valflags);
    if (value) {
        self->event_count = *(const uint32_t*)value;
        fprintf(stderr, "Romulus: Restored event_count=%u (size=%lu, type=%u)\n", 
               self->event_count, size, type);
    } else {
        fprintf(stderr, "Romulus: No event_count in state\n");
    }
    
    /* Restore loop length */
    value = retrieve(handle, loop_length_frames_urid, &size, &type, &valflags);
    if (value) {
        self->loop_length_frames = *(const uint32_t*)value;
        fprintf(stderr, "Romulus: Restored loop_length_frames=%u (size=%lu, type=%u)\n", 
               self->loop_length_frames, size, type);
    } else {
        fprintf(stderr, "Romulus: No loop_length_frames in state\n");
    }
    
    /* Restore events */
    value = retrieve(handle, events_urid, &size, &type, &valflags);
    if (value && self->event_count > 0) {
        fprintf(stderr, "Romulus: Restoring events, size=%lu bytes, type=%u\n", size, type);
        size_t copy_size = size;
        if (copy_size > sizeof(MidiEvent) * MAX_MIDI_EVENTS) {
            copy_size = sizeof(MidiEvent) * MAX_MIDI_EVENTS;
            fprintf(stderr, "Romulus: WARNING - truncating events to fit MAX_MIDI_EVENTS\n");
        }
        memcpy(self->events, value, copy_size);
        
        /* If we restored events, transition to IDLE so transport start can trigger playback */
        if (self->event_count > 0) {
            self->state = STATE_IDLE;
            fprintf(stderr, "Romulus: Restored %u events from state, transitioning to IDLE\n", self->event_count);
        }
    } else {
        fprintf(stderr, "Romulus: No events in state (value=%p, event_count=%u)\n", 
               value, self->event_count);
    }
    
    fprintf(stderr, "Romulus: State restore complete\n");
    return LV2_STATE_SUCCESS;
}

/* State interface */
static const LV2_State_Interface state_interface = {
    save,
    restore
};

/* Extension data */
static const void*
extension_data(const char* uri)
{
    if (!strcmp(uri, LV2_STATE__interface)) {
        return &state_interface;
    }
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
