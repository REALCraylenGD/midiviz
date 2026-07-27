static volatile int g_pad __attribute__((used)) = 0;
static void __attribute__((noinline)) pad_noop(int x) { g_pad = x; }
static void __attribute__((noinline)) pad1(int x) { g_pad = x; }
static void __attribute__((noinline)) pad2(int x) { g_pad = x; }
static void __attribute__((noinline)) pad3(int x) { g_pad = x; }

#include <windows.h>
#include <mmsystem.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

// ---- Software synth engine (event-driven audio thread) ----
#define SYNTH_NV 1024
#define SYNTH_SR 44100
#define SYNTH_NBUF 4
#define SYNTH_BLEN 1024

#define ATK_LEN (SYNTH_SR / 1000 * 2)
#define DEC_LEN (SYNTH_SR / 1000 * 30)
#define REL_LEN (SYNTH_SR / 1000 * 80)
#define SUS_VAL 20000
#define ENV_MAX 32767

#define WT_BITS 12
#define WT_LEN (1 << WT_BITS)

// Lock-free SPSC ring buffer for MIDI events (type: 1=on, 0=off, 2=all-off)
#define RING_SZ 4096
typedef struct { short type, key, vel; } MidiEvent;
static volatile LONG g_rw, g_rr;
static MidiEvent g_ring[RING_SZ];
static void ring_push(short type, short key, short vel) {
    LONG w = g_rw, nw = (w + 1) & (RING_SZ - 1);
    if (nw == g_rr) return;
    g_ring[w].type = type; g_ring[w].key = key; g_ring[w].vel = vel; g_rw = nw;
}
static int ring_pop(MidiEvent *ev) {
    LONG r = g_rr;
    if (r == g_rw) return 0;
    *ev = g_ring[r]; g_rr = (r + 1) & (RING_SZ - 1); return 1;
}

typedef struct {
    volatile int active, note, vel;
    volatile unsigned phase, step;
    volatile int ep, epos;
    volatile DWORD stime;
} SynthVoice;

static struct {
    SynthVoice v[SYNTH_NV];
    short wt[WT_LEN];
    HWAVEOUT hwo;
    WAVEHDR wh[SYNTH_NBUF];
    short bufs[SYNTH_NBUF][SYNTH_BLEN * 2];
    HANDLE thread, wake;
    volatile int running;
} g_syn;

static LARGE_INTEGER g_qpc_freq;

// Called from audio thread only
static void voice_on(int key, int vel) {
    int vi = -1; DWORD oldest = (DWORD)-1;
    for (int i = 0; i < SYNTH_NV; i++) {
        if (!g_syn.v[i].active) { vi = i; break; }
        if (g_syn.v[i].stime < oldest) { oldest = g_syn.v[i].stime; vi = i; }
    }
    if (vi < 0) return;
    SynthVoice *v = &g_syn.v[vi];
    v->active = 1; v->note = key; v->vel = vel > 0 ? vel : 80; v->phase = 0;
    v->step = (unsigned)((440.0 * pow(2.0, (key - 69.0) / 12.0)) * (1LL << 32) / SYNTH_SR);
    v->ep = 0; v->epos = 0; v->stime = GetTickCount();
}

static void voice_off(int key) {
    int vi = -1; DWORD newest = 0;
    for (int i = 0; i < SYNTH_NV; i++)
        if (g_syn.v[i].active && g_syn.v[i].note == key && g_syn.v[i].ep < 3)
            if (g_syn.v[i].stime >= newest) { newest = g_syn.v[i].stime; vi = i; }
    if (vi >= 0) { g_syn.v[vi].ep = 3; g_syn.v[vi].epos = 0; }
}

static void fill_buf(short *buf) {
    memset(buf, 0, SYNTH_BLEN * 4);
    for (int vi = 0; vi < SYNTH_NV; vi++) {
        SynthVoice *v = &g_syn.v[vi];
        if (!v->active) continue;
        int ep = v->ep, epos = v->epos;
        unsigned ph = v->phase, step = v->step;
        int vel = v->vel, active = 1;
        for (int s = 0; s < SYNTH_BLEN; s++) {
            epos++;
            if (ep == 0 && epos >= ATK_LEN) { ep = 1; epos = 0; }
            else if (ep == 1 && epos >= DEC_LEN) { ep = 2; epos = 0; }
            else if (ep == 3 && epos >= REL_LEN) { active = 0; break; }
            int env = ep == 0 ? ENV_MAX * epos / ATK_LEN :
                      ep == 1 ? ENV_MAX - (ENV_MAX - SUS_VAL) * epos / DEC_LEN :
                      ep == 2 ? SUS_VAL :
                      ep == 3 ? SUS_VAL * (REL_LEN - epos) / REL_LEN : 0;
            int mix = g_syn.wt[ph >> (32 - WT_BITS)] * env / ENV_MAX * vel / 127 >> 3;
            buf[s*2] += mix; buf[s*2+1] += mix;
            ph += step;
        }
        v->ep = ep; v->epos = epos; v->phase = ph;
        if (!active) v->active = 0;
    }
}

void CALLBACK syn_cb(HWAVEOUT, UINT msg, DWORD_PTR, DWORD_PTR, DWORD_PTR) {
    if (msg == WOM_DONE) SetEvent(g_syn.wake);
}

DWORD WINAPI syn_proc(LPVOID) {
    MidiEvent ev;
    while (g_syn.running) {
        WaitForSingleObject(g_syn.wake, 5);
        while (ring_pop(&ev)) {
            if (ev.type == 2) for (int i = 0; i < SYNTH_NV; i++) g_syn.v[i].active = 0;
            else if (ev.type) voice_on(ev.key, ev.vel);
            else voice_off(ev.key);
        }
        if (g_syn.hwo) for (int bi = 0; bi < SYNTH_NBUF; bi++)
            if (g_syn.wh[bi].dwFlags & WHDR_DONE) {
                fill_buf(g_syn.bufs[bi]);
                waveOutWrite(g_syn.hwo, &g_syn.wh[bi], sizeof(WAVEHDR));
            }
    }
    return 0;
}

// KDMAPI (OmniMIDI/Keppy's Driver) function pointers
static int g_use_kdmapi;
static HMODULE g_kdmapi_dll;
static int (__stdcall *kdmapi_IsAvailable)();
static int (__stdcall *kdmapi_InitStream)(void*);
static int (__stdcall *kdmapi_TermStream)();
static int (__stdcall *kdmapi_ResetStream)();
static int (__stdcall *kdmapi_SendData)(unsigned int);

static void try_kdmapi(const char *dll_name, int use_old_names) {
    HMODULE dll = LoadLibraryA(dll_name);
    if (!dll) return;
    kdmapi_IsAvailable = (void*)GetProcAddress(dll, use_old_names ? "KDMAPI_IsInitialized" : "IsKDMAPIAvailable");
    kdmapi_InitStream   = (void*)GetProcAddress(dll, use_old_names ? "KDMAPI_InitializeDriver" : "InitializeKDMAPIStream");
    kdmapi_TermStream   = (void*)GetProcAddress(dll, use_old_names ? "KDMAPI_TerminateDriver" : "TerminateKDMAPIStream");
    kdmapi_ResetStream  = (void*)GetProcAddress(dll, use_old_names ? "KDMAPI_StopPlaying" : "ResetKDMAPIStream");
    kdmapi_SendData     = (void*)GetProcAddress(dll, use_old_names ? "KDMAPI_SendDirectData" : "SendDirectData");
    if (kdmapi_SendData && kdmapi_InitStream && kdmapi_InitStream(0)) { g_kdmapi_dll = dll; g_use_kdmapi = 1; return; }
    FreeLibrary(dll);
}

// Public API — dispatches to KDMAPI or built-in synth
static void synth_note_on(int key, int vel, int chan) {
    if (g_use_kdmapi) { kdmapi_SendData((0x90 | chan) | (key << 8) | (vel << 16)); }
    else { ring_push(1, key, vel); }
}
static void synth_note_off(int key, int chan) {
    if (g_use_kdmapi) { kdmapi_SendData((0x80 | chan) | (key << 8)); }
    else { ring_push(0, key, 0); }
}
static void synth_all_off() {
    if (g_use_kdmapi) { if (kdmapi_ResetStream) kdmapi_ResetStream(); }
    else { ring_push(2, 0, 0); }
}

static void synth_init() {
    try_kdmapi("OmniMIDI.dll", 0);
    if (!g_use_kdmapi) try_kdmapi("KeppyDriver.dll", 1);
    if (g_use_kdmapi) return;
    for (int i = 0; i < WT_LEN; i++)
        g_syn.wt[i] = (short)(sin(6.283185307 * i / WT_LEN) * 32767);
    WAVEFORMATEX wf = {WAVE_FORMAT_PCM, 2, SYNTH_SR, SYNTH_SR * 4, 4, 16, 0};
    g_syn.wake = CreateEventA(0, 0, 0, 0);
    if (waveOutOpen(&g_syn.hwo, WAVE_MAPPER, &wf, (DWORD_PTR)syn_cb, (DWORD_PTR)&g_syn, CALLBACK_FUNCTION) != MMSYSERR_NOERROR)
        { g_syn.hwo = 0; return; }
    for (int i = 0; i < SYNTH_NBUF; i++) {
        memset(&g_syn.wh[i], 0, sizeof(WAVEHDR));
        g_syn.wh[i].lpData = (LPSTR)g_syn.bufs[i];
        g_syn.wh[i].dwBufferLength = SYNTH_BLEN * 4;
        waveOutPrepareHeader(g_syn.hwo, &g_syn.wh[i], sizeof(WAVEHDR));
        memset(g_syn.bufs[i], 0, SYNTH_BLEN * 4);
        waveOutWrite(g_syn.hwo, &g_syn.wh[i], sizeof(WAVEHDR));
    }
    g_syn.running = 1;
    g_syn.thread = CreateThread(0, 0, syn_proc, 0, 0, 0);
}

static void synth_close() {
    if (g_use_kdmapi) {
        if (kdmapi_ResetStream) kdmapi_ResetStream();
        if (kdmapi_TermStream) kdmapi_TermStream();
        FreeLibrary(g_kdmapi_dll); g_kdmapi_dll = 0;
        return;
    }
    g_syn.running = 0; SetEvent(g_syn.wake);
    if (g_syn.thread) { WaitForSingleObject(g_syn.thread, 1000); CloseHandle(g_syn.thread); }
    if (g_syn.wake) CloseHandle(g_syn.wake);
    if (g_syn.hwo) {
        waveOutReset(g_syn.hwo);
        for (int i = 0; i < SYNTH_NBUF; i++)
            waveOutUnprepareHeader(g_syn.hwo, &g_syn.wh[i], sizeof(WAVEHDR));
        waveOutClose(g_syn.hwo);
    }
}

#pragma pack(push, 1)
typedef struct { char id[4]; uint32_t len; uint16_t fmt, tracks, div; } MThd;
#pragma pack(pop)

typedef struct { int tick, tempo; double us, us_per_tick; } Tempo;
// misc: key(7) | vel(7) | chan(4) | track(13) | done(1)
typedef struct { int start_ms, end_ms, start_tick, end_tick; uint32_t misc; } Note;
#define NOTE_KEY(m)  ((m)->misc & 0x7F)
#define NOTE_VEL(m)  (((m)->misc >> 7) & 0x7F)
#define NOTE_CHAN(m) (((m)->misc >> 14) & 0xF)
#define NOTE_TRACK(m) (((m)->misc >> 18) & 0x1FFF)
#define NOTE_DONE(m) ((m)->misc >> 31)
#define NOTE_SET_DONE(m) ((m)->misc |= 1u << 31)
#define NOTE_PACK(k,v,c,t) ((k) | ((v) << 7) | ((c) << 14) | ((t) << 18))

typedef struct { int note, chan, start_tick, vel; } Pending;

typedef struct {
    Note *notes;
    int *order;
    int n;
    Tempo *tempos;
    int tn, tcap;
    int tpq, dur_ms, next_idx;
} MidiData;

static int read_vlq(const uint8_t **p) {
    int v = 0; uint8_t c;
    do { c = *(*p)++; v = (v << 7) | (c & 0x7F); } while (c & 0x80);
    return v;
}

static int read16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static int read24(const uint8_t *p) { return (p[0] << 16) | (p[1] << 8) | p[2]; }
static uint32_t read32(const uint8_t *p) { return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }

static int tempo_cmp(const void *a, const void *b) {
    int ta = ((Tempo*)a)->tick, tb = ((Tempo*)b)->tick;
    return ta < tb ? -1 : ta > tb ? 1 : 0;
}

static void finalize_tempos(MidiData *md) {
    // Sort tempos (they may come from multiple tracks out of order)
    qsort(md->tempos, md->tn, sizeof(Tempo), tempo_cmp);
    // Merge duplicates (keep last for same tick)
    int w = 1;
    for (int i = 1; i < md->tn; i++)
        if (md->tempos[i].tick != md->tempos[w-1].tick) md->tempos[w++] = md->tempos[i];
    md->tn = w;
    // Precompute us_per_tick and cumulative microseconds
    double us = 0;
    int prev_tick = md->tempos[0].tick;
    md->tempos[0].us = 0;
    md->tempos[0].us_per_tick = (double)md->tempos[0].tempo / md->tpq;
    for (int i = 1; i < md->tn; i++) {
        us += (double)(md->tempos[i].tick - prev_tick) * md->tempos[i-1].tempo / md->tpq;
        md->tempos[i].us = us;
        md->tempos[i].us_per_tick = (double)md->tempos[i].tempo / md->tpq;
        prev_tick = md->tempos[i].tick;
    }
}

static void radix_sort_order(int *order, int *tmp, int *keys, int n) {
    int count[65536];
    for (int shift = 0; shift < 32; shift += 16) {
        memset(count, 0, sizeof(count));
        for (int i = 0; i < n; i++) count[(keys[order[i]] >> shift) & 0xFFFF]++;
        int sum = 0;
        for (int i = 0; i < 65536; i++) { int c = count[i]; count[i] = sum; sum += c; }
        for (int i = 0; i < n; i++) tmp[count[(keys[order[i]] >> shift) & 0xFFFF]++] = order[i];
        memcpy(order, tmp, n * sizeof(int));
    }
}



static int ms_to_tick(MidiData *md, double ms) {
    double us = ms * 1000.0;
    int lo = 0, hi = md->tn - 1;
    if (us <= md->tempos[0].us) {
        return md->tempos[0].tick + (int)(us * md->tpq / md->tempos[0].tempo);
    }
    if (us >= md->tempos[hi].us) {
        return md->tempos[hi].tick + (int)((us - md->tempos[hi].us) * md->tpq / md->tempos[hi].tempo);
    }
    while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (md->tempos[mid].us <= us) lo = mid;
        else hi = mid - 1;
    }
    return md->tempos[lo].tick + (int)((us - md->tempos[lo].us) * md->tpq / md->tempos[lo].tempo);
}

typedef struct {
    const uint8_t *data;
    uint32_t len;
    int track_num;
} TrackChunk;

static volatile LONG g_threads_done;

typedef struct {
    TrackChunk *chunks;
    int num_chunks;
    Note *notes;
    int n, cap;
    Tempo *tempos;
    int tn, tcap;
    int tpq;
} ParseCtx;

// Parse: extract tempos + emit notes with tick values (no ms), shared array
static DWORD WINAPI parse_thread_proc(LPVOID param) {
    ParseCtx *ctx = (ParseCtx*)param;
    int np_cap = 16384;
    Pending *pending = malloc(np_cap * sizeof(Pending));
    ctx->n = 0;

    for (int ci = 0; ci < ctx->num_chunks; ci++) {
        const uint8_t *data = ctx->chunks[ci].data;
        uint32_t trk_len = ctx->chunks[ci].len;
        int track_num = ctx->chunks[ci].track_num;
        const uint8_t *trk_end = data + trk_len;
        int tick = 0;
        uint8_t running_status = 0;
        int np = 0;
        int active[128 * 16];
        memset(active, -1, sizeof(active));

        while (data < trk_end) {
            uint8_t _c; int delta;
            _c = *data++; delta = _c & 0x7F;
            if (_c & 0x80) { _c = *data++; delta = (delta << 7) | (_c & 0x7F); if (_c & 0x80) { _c = *data++; delta = (delta << 7) | (_c & 0x7F); if (_c & 0x80) { _c = *data++; delta = (delta << 7) | (_c & 0x7F); } } }
            tick += delta;
            if (data >= trk_end) break;
            uint8_t ev = *data;
            if (ev == 0xFF) {
                data++; uint8_t type = *data++;
                int len = 0; do { _c = *data++; len = (len << 7) | (_c & 0x7F); } while (_c & 0x80);
                if (type == 0x51 && len >= 3) {
                    if (ctx->tn >= ctx->tcap) { ctx->tcap *= 2; ctx->tempos = realloc(ctx->tempos, ctx->tcap * sizeof(Tempo)); }
                    ctx->tempos[ctx->tn].tick = tick; ctx->tempos[ctx->tn].tempo = (data[0] << 16) | (data[1] << 8) | data[2]; ctx->tn++;
                }
                data += len;
                running_status = 0;
            } else if (ev == 0xF0 || ev == 0xF7) {
                data++;
                int len = 0; do { _c = *data++; len = (len << 7) | (_c & 0x7F); } while (_c & 0x80);
                data += len;
                running_status = 0;
            } else {
                if (ev & 0x80) running_status = *data++;
                ev = running_status;
                int hi = (ev >> 4) & 0xF;
                if (hi == 0x8 || hi == 0x9 || hi == 0xA || hi == 0xB || hi == 0xE) {
                    int p1 = *data++; int p2 = *data++;
                    int chan = ev & 0xF;
                    int *slot = &active[p1 * 16 + chan];
                    if (hi == 0x9 && p2 > 0) {
                        if (*slot >= 0) {
                            int ai = *slot;
                            Note *n = &ctx->notes[ctx->n];
                            n->misc = NOTE_PACK(p1, pending[ai].vel, chan, track_num);
                            n->start_tick = pending[ai].start_tick; n->end_tick = tick; ctx->n++;
                            np--;
                            if (ai != np) { pending[ai] = pending[np]; active[pending[np].note * 16 + pending[np].chan] = ai; }
                        }
                        if (np >= np_cap) { np_cap *= 2; pending = realloc(pending, np_cap * sizeof(Pending)); }
                        *slot = np;
                        pending[np].note = p1; pending[np].chan = chan; pending[np].start_tick = tick; pending[np].vel = p2; np++;
                    } else if (hi == 0x8 || (hi == 0x9 && p2 == 0)) {
                        int ai = *slot;
                        if (ai >= 0) {
                            Note *n = &ctx->notes[ctx->n];
                            n->misc = NOTE_PACK(pending[ai].note, pending[ai].vel, pending[ai].chan, track_num);
                            n->start_tick = pending[ai].start_tick; n->end_tick = tick; ctx->n++;
                            *slot = -1; np--;
                            if (ai != np) { pending[ai] = pending[np]; active[pending[np].note * 16 + pending[np].chan] = ai; }
                        }
                    }
                } else if (hi == 0xC || hi == 0xD) { data++; }
            }
        }
        for (int i = 0; i < np; i++) {
            Note *n = &ctx->notes[ctx->n];
            n->misc = NOTE_PACK(pending[i].note, pending[i].vel, pending[i].chan, track_num);
            n->start_tick = pending[i].start_tick; n->end_tick = tick; ctx->n++;
        }
    }
    free(pending);
    InterlockedIncrement(&g_threads_done);
    return 0;
}

static MidiData *parse_midi(const uint8_t *data, int64_t size) {
    if (size < 14) return 0;
    MThd *hdr = (MThd*)data;
    int fmt = read16((void*)&hdr->fmt);
    if (memcmp(hdr->id, "MThd", 4) || fmt > 2) return 0;
    const uint8_t *end = data + size;
    data += 14;

    MidiData *md = calloc(1, sizeof(MidiData));
    md->tpq = read16((void*)&hdr->div);
    if (md->tpq & 0x8000) { free(md); return 0; }

    // ---- Collect track chunks ----
    int num_tracks = read16((void*)&hdr->tracks);
    TrackChunk *chunks = malloc(num_tracks * sizeof(TrackChunk));
    int actual = 0;
    const uint8_t *p = data;
    for (int t = 0; t < num_tracks && p + 8 <= end; t++) {
        uint32_t trk_len = read32(p + 4);
        if (memcmp(p, "MTrk", 4) == 0) {
            chunks[actual].len = trk_len;
            chunks[actual].data = p + 8;
            chunks[actual].track_num = t;
            actual++;
        }
        p += 8 + trk_len;
    }
    if (actual == 0) { free(md); free(chunks); return 0; }

    LARGE_INTEGER pf;
    QueryPerformanceFrequency(&pf);

    // ---- Distribute chunks ----
    SYSTEM_INFO sysinfo;
    GetSystemInfo(&sysinfo);
    int num_threads = sysinfo.dwNumberOfProcessors;
    if (num_threads > 6) num_threads = 6;
    if (num_threads < 1) num_threads = 1;
    if (num_threads > actual) num_threads = actual;

    for (int i = 0; i < actual; i++)
        for (int j = i + 1; j < actual; j++)
            if (chunks[j].len > chunks[i].len) {
                TrackChunk tmp = chunks[i]; chunks[i] = chunks[j]; chunks[j] = tmp;
            }

    int *counts = calloc(num_threads, sizeof(int));
    for (int i = 0; i < actual; i++) counts[i % num_threads]++;
    ParseCtx *ctxs = calloc(num_threads, sizeof(ParseCtx));
    for (int i = 0; i < num_threads; i++) {
        ctxs[i].chunks = malloc(counts[i] * sizeof(TrackChunk));
        ctxs[i].num_chunks = 0;
    }
    for (int i = 0; i < actual; i++) {
        int t = i % num_threads;
        ctxs[t].chunks[ctxs[t].num_chunks++] = chunks[i];
    }
    free(counts);

    // ---- Estimate total capacity (chunk_size/8 empirical ~= note count) ----
    int64_t total_est = 0;
    int offsets[128], caps[128];
    for (int i = 0; i < num_threads; i++) {
        int te = 0;
        for (int j = 0; j < ctxs[i].num_chunks; j++)
            te += ctxs[i].chunks[j].len / 8;
        if (te < 65536) te = 65536;
        te += te / 10; // 10% headroom
        caps[i] = te;
        offsets[i] = (int)total_est;
        total_est += te;
    }
    md->notes = malloc(total_est * sizeof(Note));

    // ---- Setup: per-thread tempo arrays + shared note slice ----
    for (int i = 0; i < num_threads; i++) {
        ctxs[i].tcap = 1024;
        ctxs[i].tempos = malloc(ctxs[i].tcap * sizeof(Tempo));
        ctxs[i].tn = 0;
        ctxs[i].tpq = md->tpq;
        ctxs[i].notes = md->notes + offsets[i];
        ctxs[i].cap = caps[i];
        ctxs[i].n = 0;
    }

    LARGE_INTEGER tp0;
    QueryPerformanceCounter(&tp0);
    HANDLE *threads = malloc(num_threads * sizeof(HANDLE));
    g_threads_done = 0;
    for (int i = 0; i < num_threads; i++)
        threads[i] = CreateThread(NULL, 0, parse_thread_proc, &ctxs[i], 0, NULL);
    WaitForMultipleObjects(num_threads, threads, TRUE, INFINITE);
    for (int i = 0; i < num_threads; i++) CloseHandle(threads[i]);
    free(threads);

    LARGE_INTEGER tp1;
    QueryPerformanceCounter(&tp1);

    // ---- Merge and finalize tempos ----
    int total_tempos = 0;
    for (int i = 0; i < num_threads; i++) total_tempos += ctxs[i].tn;
    md->tempos = malloc(total_tempos * sizeof(Tempo));
    md->tcap = total_tempos;
    md->tn = 0;
    for (int i = 0; i < num_threads; i++) {
        for (int j = 0; j < ctxs[i].tn; j++)
            md->tempos[md->tn++] = ctxs[i].tempos[j];
        free(ctxs[i].tempos);
    }
    if (md->tn == 0) {
        md->tempos = malloc(sizeof(Tempo));
        md->tempos[0].tick = 0; md->tempos[0].tempo = 500000; md->tn = 1;
    }
    finalize_tempos(md);

    LARGE_INTEGER tp2;
    QueryPerformanceCounter(&tp2);

    // ---- Pack thread slices into contiguous md->notes ----
    int total_notes = 0;
    for (int i = 0; i < num_threads; i++) total_notes += ctxs[i].n;
    md->n = total_notes;
    int dst = 0;
    for (int i = 0; i < num_threads; i++) {
        int src = offsets[i], n = ctxs[i].n;
        if (src != dst)
            memmove(md->notes + dst, md->notes + src, n * sizeof(Note));
        dst += n;
    }

    // ---- Compute ms via binary search tempo lookup ----
    for (int i = 0; i < md->n; i++) {
        int t = md->notes[i].start_tick;
        int lo = 0, hi = md->tn - 1;
        while (lo < hi) { int mid = (lo + hi + 1) >> 1; if (t >= md->tempos[mid].tick) lo = mid; else hi = mid - 1; }
        md->notes[i].start_ms = (int)((md->tempos[lo].us + (t - md->tempos[lo].tick) * md->tempos[lo].us_per_tick) * 0.001);
        t = md->notes[i].end_tick;
        while (lo + 1 < md->tn && t >= md->tempos[lo + 1].tick) lo++;
        md->notes[i].end_ms = (int)((md->tempos[lo].us + (t - md->tempos[lo].tick) * md->tempos[lo].us_per_tick) * 0.001);
        md->notes[i].misc &= ~(1u << 31); // done=0
    }
    for (int i = 0; i < num_threads; i++)
        free(ctxs[i].chunks);
    free(ctxs);

    LARGE_INTEGER ts;
    QueryPerformanceCounter(&ts);

    // ---- Sort by start_ms (index sort, keep order array) ----
    md->order = malloc(md->n * sizeof(int));
    for (int i = 0; i < md->n; i++) md->order[i] = i;
    int *keys = malloc(md->n * sizeof(int));
    for (int i = 0; i < md->n; i++) keys[i] = md->notes[i].start_ms;
    int *tmp = malloc(md->n * sizeof(int));
    radix_sort_order(md->order, tmp, keys, md->n);
    free(tmp); free(keys);
    LARGE_INTEGER te;
    QueryPerformanceCounter(&te);
    fprintf(stderr, "  parse: %.2fs  tempo: %.2fs  pack+ms: %.2fs  sort: %.2fs\n",
        (double)(tp1.QuadPart - tp0.QuadPart) / pf.QuadPart,
        (double)(tp2.QuadPart - tp1.QuadPart) / pf.QuadPart,
        (double)(ts.QuadPart - tp2.QuadPart) / pf.QuadPart,
        (double)(te.QuadPart - ts.QuadPart) / pf.QuadPart);
    fflush(stderr);

    md->dur_ms = 0;
    for (int i = 0; i < md->n; i++)
        if (md->notes[i].end_ms > md->dur_ms) md->dur_ms = md->notes[i].end_ms;
    md->dur_ms += 2000;
    free(chunks);
    return md;
}

static COLORREF chan_colors[16], track_colors[16];
static int color_by_track;
static void init_colors() {
    int cols[16][3] = {
        {255,60,60},{255,180,60},{220,255,60},{60,255,100},
        {60,255,200},{60,200,255},{80,100,255},{160,60,255},
        {230,60,255},{255,60,180},{255,80,120},{180,180,180},
        {0,200,180},{120,255,60},{200,120,255},{255,200,60}
    };
    for (int i = 0; i < 16; i++) {
        chan_colors[i] = RGB(cols[i][0], cols[i][1], cols[i][2]);
        track_colors[i] = RGB(cols[(i*7+3)%16][0], cols[(i*7+3)%16][1], cols[(i*7+3)%16][2]);
    }
}

static struct {
    MidiData *md;
    double current_ms;
    int current_tick;
    int playing;
    HWND hwnd;
    int win_w, win_h;
    HDC back_dc;
    HBITMAP back_bmp;
    int active_cnt[128];
    int render_idx, off_idx;
    LARGE_INTEGER wall_qpc;
    double pause_total_ms;
    LARGE_INTEGER pause_qpc;
    int fps, fps_frames;
    DWORD fps_last_tc;
} g;

static void all_off() {
    synth_all_off();
}

#define MIN_KEY 0
#define MAX_KEY 127
#define KEY_COUNT (MAX_KEY - MIN_KEY + 1)

static int key_to_x(int key) {
    return (key - MIN_KEY) * (g.win_w - 10) / KEY_COUNT + 5;
}

static int key_width() {
    return (g.win_w - 10) / KEY_COUNT;
}

static int is_black(int key) {
    static const int b[] = {1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1};
    return b[key % 12];
}

static void draw_keyboard(HDC dc) {
    int kw = key_width();
    int ky = g.win_h - 30;
    int kh = 28;
    
    // White keys first
    for (int k = MIN_KEY; k <= MAX_KEY; k++) {
        if (is_black(k)) continue;
        int x = key_to_x(k);
        RECT r = {x, ky, x + kw, ky + kh};
        HBRUSH br = g.active_cnt[k] ? GetStockObject(WHITE_BRUSH) : CreateSolidBrush(RGB(240, 240, 240));
        FillRect(dc, &r, br);
        if (!g.active_cnt[k]) DeleteObject(br);
        FrameRect(dc, &r, GetStockObject(BLACK_BRUSH));
    }
    // Black keys
    for (int k = MIN_KEY; k <= MAX_KEY; k++) {
        if (!is_black(k)) continue;
        int x = key_to_x(k);
        RECT r = {x - kw / 4 + 1, ky, x + kw / 4 + 1, ky + kh / 2};
        HBRUSH br = g.active_cnt[k] ? GetStockObject(WHITE_BRUSH) : GetStockObject(BLACK_BRUSH);
        FillRect(dc, &r, br);
        FrameRect(dc, &r, GetStockObject(BLACK_BRUSH));
    }
}

static void render() {
    if (!g.md || !g.hwnd) return;
    HDC dc = GetDC(g.hwnd);
    if (!dc) return;
    RECT cr; GetClientRect(g.hwnd, &cr);
    g.win_w = cr.right; g.win_h = cr.bottom;
    if (g.win_w < 1 || g.win_h < 1) { ReleaseDC(g.hwnd, dc); return; }
    
    if (!g.back_dc) {
        g.back_dc = CreateCompatibleDC(dc);
        g.back_bmp = CreateCompatibleBitmap(dc, g.win_w, g.win_h);
        if (!g.back_dc || !g.back_bmp) {
            if (g.back_dc) { DeleteDC(g.back_dc); g.back_dc = 0; }
            ReleaseDC(g.hwnd, dc); return;
        }
        SelectObject(g.back_dc, g.back_bmp);
    }
    
    HDC bdc = g.back_dc;
    RECT br = {0, 0, g.win_w, g.win_h};
    FillRect(bdc, &br, GetStockObject(BLACK_BRUSH));
    
    int hit_y = g.win_h - 35;
    
    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(30, 30, 30));
    SelectObject(bdc, grid_pen);
    for (int k = MIN_KEY; k <= MAX_KEY; k++) {
        int x = key_to_x(k);
        if (is_black(k)) continue;
        MoveToEx(bdc, x, 0, 0); LineTo(bdc, x, hit_y);
    }
    DeleteObject(grid_pen);
    
    int max_future = (int)(g.current_ms + 1200);
    int min_past = (int)(g.current_ms - 500);
    int curr_tick = g.current_tick;
    int kw = key_width();
    int init_tempo = g.md->tempos[0].tempo;
    int lookahead_ticks = (int)(1200.0 * 1000.0 * g.md->tpq / init_tempo);
    if (lookahead_ticks < 1) lookahead_ticks = g.md->tpq;
    int *order = g.md->order;
    while (g.render_idx < g.md->n && g.md->notes[order[g.render_idx]].end_ms < min_past)
        g.render_idx++;
    for (int i = g.render_idx; i < g.md->n; i++) {
        Note *n = &g.md->notes[order[i]];
        if (n->start_ms > max_future) break;
        int y_start = hit_y - (int)((double)(n->start_tick - curr_tick) / lookahead_ticks * hit_y);
        int y_end   = hit_y - (int)((double)(n->end_tick   - curr_tick) / lookahead_ticks * hit_y);
        int y_top = y_end, y_bot = y_start;
        if (y_bot < 0 || y_top > g.win_h) continue;
        if (y_top < 0) y_top = 0;
        if (y_bot > g.win_h) y_bot = g.win_h;
        if (y_bot - y_top < 2 && y_top + 3 < g.win_h) y_bot = y_top + 3;
        
        int x = key_to_x(NOTE_KEY(n));
        RECT r = {x + 1, y_top, x + kw - 1, y_bot};
        HBRUSH br = CreateSolidBrush(color_by_track ? track_colors[NOTE_TRACK(n) & 15] : chan_colors[NOTE_CHAN(n) & 15]);
        FillRect(bdc, &r, br);
        DeleteObject(br);
    }
    
    draw_keyboard(bdc);
    
    SetBkMode(bdc, TRANSPARENT);
    SetTextColor(bdc, RGB(200, 200, 200));
    char buf[256];
    int sec = (int)(g.current_ms / 1000);
    int ds = g.md->dur_ms / 1000;
    int ms_disp = (int)g.current_ms % 1000;
    g.fps_frames++;
    DWORD tc = GetTickCount();
    if (tc - g.fps_last_tc >= 1000) {
        g.fps = g.fps_frames;
        g.fps_frames = 0;
        g.fps_last_tc = tc;
    }
    snprintf(buf, sizeof(buf), "%02d:%02d.%03d / %02d:%02d  %s  %d FPS  Notes: %d/%d  %s",
        sec / 60, sec % 60, ms_disp, ds / 60, ds % 60,
        g.playing ? "PLAY" : "PAUSED",
        g.fps, g.md->next_idx, g.md->n,
        color_by_track ? "TRACK" : "CHAN");
    TextOutA(bdc, 10, 4, buf, strlen(buf));
    
    HPEN hit_pen = CreatePen(PS_SOLID, 2, RGB(60, 200, 255));
    SelectObject(bdc, hit_pen);
    MoveToEx(bdc, 0, hit_y, 0); LineTo(bdc, g.win_w, hit_y);
    DeleteObject(hit_pen);
    
    BitBlt(dc, 0, 0, g.win_w, g.win_h, bdc, 0, 0, SRCCOPY);
    ReleaseDC(g.hwnd, dc);
}

static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l) {
    switch (msg) {
    case WM_CREATE:
        g.hwnd = hwnd;
        SetTimer(hwnd, 1, 3, 0);
        return 0;
    case WM_TIMER:
        if (g.playing && g.md) {
            LARGE_INTEGER now;
            QueryPerformanceCounter(&now);
            g.current_ms = ((double)(now.QuadPart - g.wall_qpc.QuadPart) * 1000.0 / g_qpc_freq.QuadPart) - g.pause_total_ms;
            g.current_tick = ms_to_tick(g.md, g.current_ms);
            int *next = &g.md->next_idx;
            int *order = g.md->order;
            Note *notes = g.md->notes;
            while (*next < g.md->n && notes[order[*next]].start_ms <= g.current_ms) {
                Note *n = &notes[order[*next]];
                synth_note_on(NOTE_KEY(n), NOTE_VEL(n), NOTE_CHAN(n));
                g.active_cnt[NOTE_KEY(n)]++;
                (*next)++;
            }
            int max_check = g.md->next_idx;
            while (g.off_idx < max_check && NOTE_DONE(&notes[order[g.off_idx]])) g.off_idx++;
            for (int i = g.off_idx; i < max_check; i++) {
                int oi = order[i];
                Note *n = &notes[oi];
                if (NOTE_DONE(n)) continue;
                if (n->end_ms > 0 && n->end_ms <= g.current_ms) {
                    NOTE_SET_DONE(n);
                    g.active_cnt[NOTE_KEY(n)]--;
                    synth_note_off(NOTE_KEY(n), NOTE_CHAN(n));
                }
            }
            if (g.current_ms > g.md->dur_ms) { g.playing = 0; all_off(); memset(g.active_cnt, 0, sizeof(g.active_cnt)); }
        }
        render();
        return 0;
    case WM_PAINT: { PAINTSTRUCT ps; BeginPaint(hwnd, &ps); render(); EndPaint(hwnd, &ps); return 0; }
    case WM_SIZE:
        if (g.back_bmp) { DeleteObject(g.back_bmp); g.back_bmp = 0; DeleteDC(g.back_dc); g.back_dc = 0; }
        return 0;
    case WM_KEYDOWN:
        if (w == VK_SPACE) {
            if (g.playing) {
                QueryPerformanceCounter(&g.pause_qpc);
                g.playing = 0;
                all_off();
            } else {
                LARGE_INTEGER now;
                QueryPerformanceCounter(&now);
                g.pause_total_ms += (double)(now.QuadPart - g.pause_qpc.QuadPart) * 1000.0 / g_qpc_freq.QuadPart;
                g.playing = 1;
            }
        }
        if (w == 'R') {
            all_off(); memset(g.active_cnt, 0, sizeof(g.active_cnt));
            g.current_ms = 0; g.current_tick = 0; g.md->next_idx = 0; g.render_idx = 0; g.off_idx = 0;
            for (int i = 0; i < g.md->n; i++) g.md->notes[g.md->order[i]].misc &= ~(1u << 31);
            QueryPerformanceCounter(&g.wall_qpc); g.pause_total_ms = 0;
            g.playing = 1;
        }
        if (w == 'C') { color_by_track ^= 1; }
        return 0;
    case WM_DESTROY:
        g.playing = 0; all_off();
        synth_close();
        KillTimer(hwnd, 1);
        if (g.back_bmp) DeleteObject(g.back_bmp);
        if (g.back_dc) DeleteDC(g.back_dc);
        if (g.md) { free(g.md->notes); free(g.md->order); free(g.md->tempos); free(g.md); }
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcA(hwnd, msg, w, l);
}

// ---- Streaming mode ---- 
typedef struct { int note, chan, start_tick, vel; } SPending;
typedef struct { int type, key, vel, chan, tick; } SNoteEvent;

typedef struct {
    const uint8_t *data, *end;
    int tick, running_status;
    int np, np_cap;
    SPending *pending;
    int active[128 * 16];
    int done;
} STrackState;

typedef struct { int start_ms, end_ms, start_tick, end_tick, key, vel, chan, track; } SVisNote;

static Tempo *s_tempos;
static int s_tn, s_tcap, s_tpq;
static int s_actual;
static uint32_t *s_trk_lens;
static const uint8_t **s_trk_data;
static STrackState *s_tracks;
static SVisNote *s_vis;
static int s_vis_count, s_vis_cap;
static double s_current_ms, s_pause_total_ms;
static int s_current_tick, s_dur_ms;
static LARGE_INTEGER s_wall_qpc, s_pause_qpc;
static int s_playing;

static int s_parse_one_event(STrackState *t, Tempo *out_tempo, SNoteEvent *ne) {
    uint8_t _c; int delta;
    _c = *t->data++; delta = _c & 0x7F;
    if (_c & 0x80) { _c = *t->data++; delta = (delta << 7) | (_c & 0x7F); if (_c & 0x80) { _c = *t->data++; delta = (delta << 7) | (_c & 0x7F); if (_c & 0x80) { _c = *t->data++; delta = (delta << 7) | (_c & 0x7F); } } }
    t->tick += delta;
    if (t->data >= t->end) return 0;
    uint8_t ev = *t->data;
    if (ev == 0xFF) {
        t->data++; uint8_t type = *t->data++;
        int len = 0; do { _c = *t->data++; len = (len << 7) | (_c & 0x7F); } while (_c & 0x80);
        if (type == 0x51 && len >= 3 && out_tempo) { out_tempo->tick = t->tick; out_tempo->tempo = read24(t->data); }
        t->data += len; t->running_status = 0;
    } else if (ev == 0xF0 || ev == 0xF7) {
        t->data++; int len = 0; do { _c = *t->data++; len = (len << 7) | (_c & 0x7F); } while (_c & 0x80);
        t->data += len; t->running_status = 0;
    } else {
        if (ev & 0x80) t->running_status = *t->data++;
        ev = t->running_status; int hi = (ev >> 4) & 0xF; int chan = ev & 0xF;
        if (hi == 0x8 || hi == 0x9 || hi == 0xA || hi == 0xB || hi == 0xE) {
            int p1 = *t->data++; int p2 = *t->data++;
            int *slot = &t->active[p1 * 16 + chan];
            if (hi == 0x9 && p2 > 0) {
                if (t->np >= t->np_cap) { t->np_cap = t->np_cap ? t->np_cap * 2 : 1024; t->pending = realloc(t->pending, t->np_cap * sizeof(SPending)); }
                *slot = t->np; t->pending[t->np].note = p1; t->pending[t->np].chan = chan;
                t->pending[t->np].start_tick = t->tick; t->pending[t->np].vel = p2; t->np++;
                if (ne) { ne->type = 1; ne->key = p1; ne->vel = p2; ne->chan = chan; ne->tick = t->tick; }
                return 1;
            } else {
                int ai = *slot;
                if (ai >= 0) {
                    if (ne) { ne->type = -1; ne->key = t->pending[ai].note; ne->vel = t->pending[ai].vel; ne->chan = t->pending[ai].chan; ne->tick = t->tick; }
                    *slot = -1; t->np--;
                    if (ai != t->np) { int mk = t->pending[t->np].note, mc = t->pending[t->np].chan; t->pending[ai] = t->pending[t->np]; t->active[mk*16+mc] = ai; }
                    return -1;
                }
            }
        } else if (hi == 0xC || hi == 0xD) { t->data++; }
    }
    return 0;
}

static int s_tick_to_ms(int tick) {
    if (s_tn == 0) return (int)((double)tick * 500000.0 / s_tpq / 1000.0);
    int lo = 0, hi = s_tn - 1;
    if (tick <= s_tempos[0].tick) return (int)((double)tick * s_tempos[0].us_per_tick * 0.001);
    if (tick >= s_tempos[hi].tick) return (int)((s_tempos[hi].us + (double)(tick - s_tempos[hi].tick) * s_tempos[hi].us_per_tick) * 0.001);
    while (lo < hi) { int mid = (lo + hi + 1) >> 1; if (tick >= s_tempos[mid].tick) lo = mid; else hi = mid - 1; }
    return (int)((s_tempos[lo].us + (double)(tick - s_tempos[lo].tick) * s_tempos[lo].us_per_tick) * 0.001);
}

static int s_ms_to_tick(double ms) {
    double us = ms * 1000.0;
    if (s_tn == 0) return (int)(us * s_tpq / 500000.0);
    int lo = 0, hi = s_tn - 1;
    if (us <= s_tempos[0].us) return s_tempos[0].tick + (int)(us * s_tpq / s_tempos[0].tempo);
    if (us >= s_tempos[hi].us) return s_tempos[hi].tick + (int)((us - s_tempos[hi].us) * s_tpq / s_tempos[hi].tempo);
    while (lo < hi) { int mid = (lo + hi + 1) >> 1; if (us >= s_tempos[mid].us) lo = mid; else hi = mid - 1; }
    return s_tempos[lo].tick + (int)((us - s_tempos[lo].us) * s_tpq / s_tempos[lo].tempo);
}

static void s_advance_to_ms(double target_ms) {
    int target_tick; double us = target_ms * 1000.0;
    if (s_tn == 0) target_tick = (int)(target_ms * 1000.0 * s_tpq / 500000.0);
    else {
        int lo = 0, hi = s_tn - 1;
        if (us <= s_tempos[0].us) target_tick = s_tempos[0].tick + (int)(us * s_tpq / s_tempos[0].tempo);
        else if (us >= s_tempos[hi].us) target_tick = s_tempos[hi].tick + (int)((us - s_tempos[hi].us) * s_tpq / s_tempos[hi].tempo);
        else { while (lo < hi) { int mid = (lo + hi + 1) >> 1; if (us >= s_tempos[mid].us) lo = mid; else hi = mid - 1; } target_tick = s_tempos[lo].tick + (int)((us - s_tempos[lo].us) * s_tpq / s_tempos[lo].tempo); }
    }
    // Save state, visual pass, restore, audio pass
    STrackState *saved = malloc(s_actual * sizeof(STrackState));
    memcpy(saved, s_tracks, s_actual * sizeof(STrackState));
    for (int t = 0; t < s_actual; t++) {
        saved[t].pending = s_tracks[t].np_cap ? malloc(s_tracks[t].np_cap * sizeof(SPending)) : 0;
        if (saved[t].pending) memcpy(saved[t].pending, s_tracks[t].pending, s_tracks[t].np_cap * sizeof(SPending));
    }
    for (int t = 0; t < s_actual; t++) {
        STrackState *tr = &s_tracks[t]; if (tr->done) continue; SNoteEvent ne;
        while (tr->tick < target_tick && tr->data < tr->end) {
            int r = s_parse_one_event(tr, 0, &ne);
            if (r == 1 && s_vis_count < s_vis_cap) {
                SVisNote *v = &s_vis[s_vis_count++]; v->start_ms = s_tick_to_ms(ne.tick);
                v->end_ms = v->start_ms + 200; v->start_tick = ne.tick; v->end_tick = ne.tick + 96;
                v->key = ne.key; v->vel = ne.vel; v->chan = ne.chan; v->track = t;
            } else if (r == -1) {
                for (int i = s_vis_count - 1; i >= 0; i--) {
                    SVisNote *v = &s_vis[i]; if (v->key == ne.key && v->chan == ne.chan && v->end_tick == v->start_tick + 96) { v->end_ms = s_tick_to_ms(ne.tick); v->end_tick = ne.tick; break; }
                }
            }
        }
    }
    for (int t = 0; t < s_actual; t++) { free(s_tracks[t].pending); s_tracks[t] = saved[t]; } free(saved);
    for (int t = 0; t < s_actual; t++) {
        STrackState *tr = &s_tracks[t]; if (tr->done) continue; SNoteEvent ne;
        while (tr->tick < s_current_tick && tr->data < tr->end) {
            int r = s_parse_one_event(tr, 0, &ne);
            if (r == 1) synth_note_on(ne.key, ne.vel, ne.chan);
            else if (r == -1) synth_note_off(ne.key, ne.chan);
        }
        if (tr->data >= tr->end) {
            for (int i = 0; i < tr->np; i++) synth_note_off(tr->pending[i].note, tr->pending[i].chan);
            tr->np = 0; tr->done = 1;
        }
    }
}

static struct { HWND hwnd; int win_w, win_h; HDC back_dc; HBITMAP back_bmp; int active_cnt[128]; int fps, fps_frames; DWORD fps_last_tc; } s_gui;
static int s_color_by_track;

static void s_render() {
    if (!s_gui.hwnd) return;
    HDC dc = GetDC(s_gui.hwnd); if (!dc) return;
    RECT cr; GetClientRect(s_gui.hwnd, &cr);
    s_gui.win_w = cr.right; s_gui.win_h = cr.bottom;
    if (s_gui.win_w < 1 || s_gui.win_h < 1) { ReleaseDC(s_gui.hwnd, dc); return; }
    if (!s_gui.back_dc) {
        s_gui.back_dc = CreateCompatibleDC(dc); s_gui.back_bmp = CreateCompatibleBitmap(dc, s_gui.win_w, s_gui.win_h);
        if (!s_gui.back_dc || !s_gui.back_bmp) { if (s_gui.back_dc) DeleteDC(s_gui.back_dc); s_gui.back_dc = 0; ReleaseDC(s_gui.hwnd, dc); return; }
        SelectObject(s_gui.back_dc, s_gui.back_bmp);
    }
    HDC bdc = s_gui.back_dc; RECT br = {0,0,s_gui.win_w,s_gui.win_h}; FillRect(bdc, &br, GetStockObject(BLACK_BRUSH));
    int hit_y = s_gui.win_h - 35;
    HPEN gp = CreatePen(PS_SOLID,1,RGB(30,30,30)); SelectObject(bdc, gp);
    for (int k = 0; k <= 127; k++) { if (is_black(k)) continue; int x = key_to_x(k); MoveToEx(bdc, x, 0, 0); LineTo(bdc, x, hit_y); }
    DeleteObject(gp);
    int kw = key_width();
    int curr_tick = s_current_tick;
    int init_tempo = s_tn > 0 ? s_tempos[0].tempo : 500000;
    int lookahead_ticks = (int)(1200.0 * 1000.0 * s_tpq / init_tempo);
    if (lookahead_ticks < 1) lookahead_ticks = s_tpq;
    for (int i = 0; i < s_vis_count; i++) {
        SVisNote *vn = &s_vis[i];
        if (vn->end_tick <= curr_tick - lookahead_ticks/2) continue;
        if (vn->start_tick > curr_tick + lookahead_ticks) continue;
        int ys = hit_y - (int)((double)(vn->start_tick - curr_tick) / lookahead_ticks * hit_y);
        int ye = hit_y - (int)((double)(vn->end_tick - curr_tick) / lookahead_ticks * hit_y);
        int yt = ye, yb = ys; if (yb < 0 || yt > s_gui.win_h) continue;
        if (yt < 0) yt = 0; if (yb > s_gui.win_h) yb = s_gui.win_h;
        if (yb - yt < 2 && yt + 3 < s_gui.win_h) yb = yt + 3;
        int x = key_to_x(vn->key); RECT r = {x+1, yt, x+kw-1, yb};
        HBRUSH br = CreateSolidBrush(s_color_by_track ? track_colors[vn->track & 15] : chan_colors[vn->chan & 15]);
        FillRect(bdc, &r, br); DeleteObject(br);
    }
    draw_keyboard(bdc);
    SetBkMode(bdc, TRANSPARENT); SetTextColor(bdc, RGB(200,200,200));
    char buf[256]; int sec = (int)(s_current_ms/1000); int ds = s_dur_ms/1000;
    s_gui.fps_frames++;
    DWORD tc = GetTickCount();
    if (tc - s_gui.fps_last_tc >= 1000) { s_gui.fps = s_gui.fps_frames; s_gui.fps_frames = 0; s_gui.fps_last_tc = tc; }
    snprintf(buf,sizeof(buf),"%02d:%02d.%03d / %02d:%02d  %s  %d FPS  Notes:%d", sec/60, sec%60, (int)s_current_ms%1000, ds/60, ds%60, s_playing?"PLAY":"PAUSED", s_gui.fps, s_vis_count);
    TextOutA(bdc, 10, 4, buf, strlen(buf));
    HPEN hp = CreatePen(PS_SOLID,2,RGB(60,200,255)); SelectObject(bdc, hp);
    MoveToEx(bdc, 0, hit_y, 0); LineTo(bdc, s_gui.win_w, hit_y); DeleteObject(hp);
    BitBlt(dc, 0, 0, s_gui.win_w, s_gui.win_h, bdc, 0, 0, SRCCOPY);
    ReleaseDC(s_gui.hwnd, dc);
}

static LRESULT CALLBACK s_WndProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l) {
    switch (msg) {
    case WM_CREATE: s_gui.hwnd = hwnd; SetTimer(hwnd, 1, 3, 0); return 0;
    case WM_TIMER:
        if (s_playing) {
            LARGE_INTEGER now; QueryPerformanceCounter(&now);
            s_current_ms = ((double)(now.QuadPart - s_wall_qpc.QuadPart) * 1000.0 / g_qpc_freq.QuadPart) - s_pause_total_ms;
            if (s_current_ms > s_dur_ms) { s_playing = 0; synth_all_off(); memset(s_gui.active_cnt, 0, sizeof(s_gui.active_cnt)); }
            s_current_tick = s_ms_to_tick(s_current_ms);
            s_advance_to_ms(s_current_ms + 1200);
        }
        s_render(); return 0;
    case WM_PAINT: { PAINTSTRUCT ps; BeginPaint(hwnd, &ps); s_render(); EndPaint(hwnd, &ps); return 0; }
    case WM_SIZE:
        if (s_gui.back_bmp) { DeleteObject(s_gui.back_bmp); s_gui.back_bmp = 0; DeleteDC(s_gui.back_dc); s_gui.back_dc = 0; }
        return 0;
    case WM_KEYDOWN:
        if (w == VK_SPACE) {
            if (s_playing) { QueryPerformanceCounter(&s_pause_qpc); s_playing = 0; synth_all_off(); memset(s_gui.active_cnt,0,sizeof(s_gui.active_cnt)); }
            else { LARGE_INTEGER now; QueryPerformanceCounter(&now); s_pause_total_ms += (double)(now.QuadPart - s_pause_qpc.QuadPart) * 1000.0 / g_qpc_freq.QuadPart; s_playing = 1; }
        }
        if (w == 'R') {
            synth_all_off(); memset(s_gui.active_cnt,0,sizeof(s_gui.active_cnt));
            for (int t = 0; t < s_actual; t++) { STrackState *tr = &s_tracks[t]; tr->data = s_trk_data[t]; tr->tick = 0; tr->running_status = 0; tr->np = 0; tr->done = 0; memset(tr->active, -1, sizeof(tr->active)); }
            s_current_ms = 0; s_current_tick = 0; s_vis_count = 0;
            QueryPerformanceCounter(&s_wall_qpc); s_pause_total_ms = 0; s_playing = 1;
        }
        if (w == 'C') s_color_by_track ^= 1;
        return 0;
    case WM_DESTROY:
        s_playing = 0; synth_all_off(); synth_close(); KillTimer(hwnd, 1);
        if (s_gui.back_bmp) DeleteObject(s_gui.back_bmp);
        if (s_gui.back_dc) DeleteDC(s_gui.back_dc);
        PostQuitMessage(0); return 0;
    }
    return DefWindowProcA(hwnd, msg, w, l);
}

static void stream_mode(const uint8_t *buf, int64_t size, int argc, char **argv) {
    if (size < 14 || memcmp(buf, "MThd", 4)) { MessageBoxA(0, "Not a MIDI file", "Error", MB_ICONERROR); return; }
    MThd *hdr = (MThd*)buf;
    if (read16((void*)&hdr->fmt) > 2) { MessageBoxA(0, "Unsupported MIDI format", "Error", MB_ICONERROR); return; }
    s_tpq = read16((void*)&hdr->div);
    if (s_tpq & 0x8000) { MessageBoxA(0, "No SMPTE support", "Error", MB_ICONERROR); return; }
    int num_tracks = read16((void*)&hdr->tracks);
    const uint8_t *p = buf + 14, *end = buf + size;
    s_trk_lens = malloc(num_tracks * sizeof(uint32_t)); s_trk_data = malloc(num_tracks * sizeof(const uint8_t*)); s_actual = 0;
    for (int t = 0; t < num_tracks && p + 8 <= end; t++) {
        uint32_t trk_len = read32(p + 4);
        if (memcmp(p, "MTrk", 4) == 0) { s_trk_data[s_actual] = p + 8; s_trk_lens[s_actual] = trk_len; s_actual++; }
        p += 8 + trk_len;
    }
    LARGE_INTEGER t0, t1; QueryPerformanceCounter(&t0);
    // Tempo prescan
    s_tcap = 1024; s_tempos = malloc(s_tcap * sizeof(Tempo)); s_tn = 0;
    int last_tick = 0;
    for (int t = 0; t < s_actual; t++) {
        STrackState ts; ts.data = s_trk_data[t]; ts.end = ts.data + s_trk_lens[t]; ts.tick = 0; ts.running_status = 0; ts.np = 0; ts.np_cap = 0; ts.pending = 0;
        memset(ts.active, -1, sizeof(ts.active));
        while (ts.data < ts.end) { Tempo tp; tp.tick = -1; s_parse_one_event(&ts, &tp, 0); if (tp.tick >= 0) { if (s_tn >= s_tcap) { s_tcap *= 2; s_tempos = realloc(s_tempos, s_tcap * sizeof(Tempo)); } s_tempos[s_tn++] = tp; } }
        if (ts.tick > last_tick) last_tick = ts.tick; free(ts.pending);
    }
    if (s_tn == 0) { s_tempos = malloc(sizeof(Tempo)); s_tempos[0].tick = 0; s_tempos[0].tempo = 500000; s_tn = 1; }
    qsort(s_tempos, s_tn, sizeof(Tempo), tempo_cmp);
    int w = 1; for (int i = 1; i < s_tn; i++) if (s_tempos[i].tick != s_tempos[w-1].tick) s_tempos[w++] = s_tempos[i]; s_tn = w;
    double us = 0; int prev = s_tempos[0].tick; s_tempos[0].us = 0; s_tempos[0].us_per_tick = (double)s_tempos[0].tempo / s_tpq;
    for (int i = 1; i < s_tn; i++) { us += (double)(s_tempos[i].tick - prev) * s_tempos[i-1].tempo / s_tpq; s_tempos[i].us = us; s_tempos[i].us_per_tick = (double)s_tempos[i].tempo / s_tpq; prev = s_tempos[i].tick; }
    s_dur_ms = s_tick_to_ms(last_tick) + 2000;
    QueryPerformanceCounter(&t1);
    int bench = 0; for (int i = 2; i < argc; i++) if (strcmp(argv[i], "--bench") == 0) bench = 1;
    if (bench) {
        fprintf(stderr, "  scan: %.2fs  tracks=%d tempos=%d dur=%ds\n", (double)(t1.QuadPart - t0.QuadPart) / g_qpc_freq.QuadPart, s_actual, s_tn, s_dur_ms/1000); fflush(stderr);
        return;
    }
    s_tracks = calloc(s_actual, sizeof(STrackState));
    for (int t = 0; t < s_actual; t++) {
        s_tracks[t].data = s_trk_data[t]; s_tracks[t].end = s_trk_data[t] + s_trk_lens[t];
        s_tracks[t].np_cap = 1024; s_tracks[t].pending = malloc(s_tracks[t].np_cap * sizeof(SPending));
        memset(s_tracks[t].active, -1, sizeof(s_tracks[t].active));
    }
    int64_t total_est = 0; for (int t = 0; t < s_actual; t++) total_est += s_trk_lens[t] / 8;
    if (total_est < 65536) total_est = 65536; total_est += total_est / 10;
    s_vis_cap = (int)(total_est > 2000000 ? 2000000 : total_est); s_vis = malloc(s_vis_cap * sizeof(SVisNote));
    init_colors(); synth_init();
    HINSTANCE hInst = GetModuleHandleA(0);
    WNDCLASSEXA wc = {sizeof(WNDCLASSEXA),0,s_WndProc,0,0,hInst,0,0,0,0,"MidiVizStream",0};
    RegisterClassExA(&wc);
    RECT wr = {0,0,1600,900}; AdjustWindowRectEx(&wr, WS_OVERLAPPEDWINDOW, 0, 0);
    HWND hwnd = CreateWindowExA(0,"MidiVizStream","MIDI Viz (Streaming)",WS_OVERLAPPEDWINDOW,CW_USEDEFAULT,CW_USEDEFAULT,wr.right-wr.left,wr.bottom-wr.top,0,0,hInst,0);
    if (!hwnd) { MessageBoxA(0,"Cannot create window","Error",MB_ICONERROR); return; }
    ShowWindow(hwnd, SW_SHOW);
    fprintf(stderr, "load: %.2fs  tracks=%d tempos=%d dur=%ds\n", (double)(t1.QuadPart - t0.QuadPart) / g_qpc_freq.QuadPart, s_actual, s_tn, s_dur_ms/1000); fflush(stderr);
    QueryPerformanceCounter(&s_wall_qpc); s_playing = 1;
    MSG msg; while (GetMessageA(&msg, 0, 0, 0)) { TranslateMessage(&msg); DispatchMessageA(&msg); }
}

int main(int argc, char **argv) {
    (void)pad_noop; (void)pad1; (void)pad2; (void)pad3;
    if (argc < 2) { MessageBoxA(0, "Usage: midiviz.exe <file.mid> [--stream] [--bench]", "Error", MB_ICONERROR); return 1; }
    
    int stream = 0;
    for (int i = 2; i < argc; i++) if (strcmp(argv[i], "--stream") == 0) stream = 1;
    
    HANDLE hFile = CreateFileA(argv[1], GENERIC_READ, FILE_SHARE_READ, 0, OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, 0);
    if (hFile == INVALID_HANDLE_VALUE) { MessageBoxA(0, "Cannot open file", "Error", MB_ICONERROR); return 1; }
    LARGE_INTEGER li; GetFileSizeEx(hFile, &li);
    HANDLE hMap = CreateFileMappingA(hFile, 0, PAGE_READONLY, li.HighPart, li.LowPart, 0);
    if (!hMap) { CloseHandle(hFile); MessageBoxA(0, "Cannot map file", "Error", MB_ICONERROR); return 1; }
    uint8_t *buf = MapViewOfFile(hMap, FILE_MAP_READ, 0, 0, 0);
    if (!buf) { CloseHandle(hMap); CloseHandle(hFile); MessageBoxA(0, "Cannot view file", "Error", MB_ICONERROR); return 1; }
    
    if (stream) { QueryPerformanceFrequency(&g_qpc_freq); stream_mode(buf, li.QuadPart, argc, argv); UnmapViewOfFile(buf); CloseHandle(hMap); CloseHandle(hFile); return 0; }
    
    LARGE_INTEGER t0, t1, t2, freq;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t0);
    g.md = parse_midi(buf, li.QuadPart);
    QueryPerformanceCounter(&t1);
    UnmapViewOfFile(buf); CloseHandle(hMap); CloseHandle(hFile);
    if (!g.md) { MessageBoxA(0, "No MIDI data found", "Error", MB_ICONERROR); return 1; }
    QueryPerformanceCounter(&t2);
    fprintf(stderr, "parse total: %.2fs  notes=%d tempos=%d\n",
        (double)(t1.QuadPart - t0.QuadPart) / freq.QuadPart,
        g.md->n, g.md->tn);
    fflush(stderr);
    
    init_colors();
    synth_init();
    timeBeginPeriod(1);
    QueryPerformanceFrequency(&g_qpc_freq);
    
    HINSTANCE hInst = GetModuleHandleA(0);
    WNDCLASSEXA wc = {sizeof(WNDCLASSEXA),0,WndProc,0,0,hInst,0,0,0,0,"MidiViz",0};
    RegisterClassExA(&wc);
    HWND hwnd = CreateWindowExA(0, "MidiViz", "MIDI Visualizer - Space=pause R=restart",
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 1000, 700, 0, 0, hInst, 0);
    ShowWindow(hwnd, SW_SHOW);
    PostMessageA(hwnd, WM_NULL, 0, 0);
    QueryPerformanceCounter(&g.wall_qpc);
    g.playing = 1;
    
    MSG msg;
    while (GetMessageA(&msg, 0, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
    return 0;
}
