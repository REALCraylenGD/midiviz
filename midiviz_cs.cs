using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;

public class MidiEngine
{
    [DllImport("winmm.dll")] static extern int midiOutOpen(out IntPtr h, int devId, IntPtr cb, IntPtr inst, int flags);
    [DllImport("winmm.dll")] static extern int midiOutClose(IntPtr h);
    [DllImport("winmm.dll")] static extern int midiOutShortMsg(IntPtr h, uint msg);
    [DllImport("kernel32.dll")] static extern bool QueryPerformanceFrequency(out long f);
    [DllImport("kernel32.dll")] static extern bool QueryPerformanceCounter(out long c);
    [DllImport("kernel32.dll", CharSet = CharSet.Auto)]
    static extern IntPtr LoadLibrary(string lp);
    [DllImport("kernel32.dll", CharSet = CharSet.Ansi)]
    static extern IntPtr GetProcAddress(IntPtr h, string name);

    delegate int KDMAPI_Init();
    delegate int KDMAPI_SendDirect(uint msg);
    delegate int KDMAPI_Terminate();
    static bool useKdmapi;
    static KDMAPI_SendDirect kdmapiSend;

    static void MidiSend(uint msg) {
        byte st = (byte)(msg & 0xFF);
        if (st < 0x80 || st > 0xEF) { if (st < 0xF0 || st > 0xFF) return; }
        if (useKdmapi) kdmapiSend(msg);
        else midiOutShortMsg(midiH, msg);
    }

    struct TState {
        public int audPos, visPos, end;
        public long audTick, visTick;
        public byte audStatus, visStatus;
        public bool active;
    }
    struct Tempo { public long tick; public int micros; public long cumulUs; }
    class TempoComparer : Comparer<Tempo> {
        public override int Compare(Tempo a, Tempo b) { return a.tick.CompareTo(b.tick); }
    }

    public struct Note { public int startTick, endTick, key, vel, chan, track; public bool done; }

    public static bool useTrackColor;

    static long qpcFreq;
    static int division;
    static int nTracks;
    static byte[] data;
    static TState[] tracks;
    static Tempo[] tempos;
    static int nTempos;
    static IntPtr midiH;
    static long startQpc;
    static volatile bool playing;
    public static bool bench;
    public static string error;
    public static string lastError;
    static int visEventsThisFrame;
    static int midiEventsThisFrame;
    static long lastFrameMs;
    static string dbgStage = "";

    static Note[] vis = new Note[262144];
    public static int visN;
    public const int MAX_VIS = 262144;
    public const int VIS_CAP = int.MaxValue;
    const int LOOKAHEAD_TICKS = 480;
    public static int curLookaheadTicks;
    static int[] compactRemap = new int[MAX_VIS];

    // Fast note lookup: (key<<8|chan) -> index in vis[]
    static Dictionary<int, int> activeNoteMap = new Dictionary<int, int>();

    public static int curMs;
    public static int curTick;

    public static Note[] VisArr { get { return vis; } }
    public static int VisCount { get { return visN; } }
    public static int TotalMs { get; private set; }
    public static bool Playing { get { return playing; } set { playing = value; } }

    public static void Load(string path)
    {
        QueryPerformanceFrequency(out qpcFreq);
        using (var fs = new FileStream(path, FileMode.Open, FileAccess.Read))
        using (var mmf = MemoryMappedFile.CreateFromFile(fs, null, 0, MemoryMappedFileAccess.Read,
            HandleInheritability.None, false))
        using (var acc = mmf.CreateViewAccessor(0, 0, MemoryMappedFileAccess.Read))
        {
            data = new byte[fs.Length];
            acc.ReadArray(0, data, 0, data.Length);
        }
        ReadHeader();
        PrescanTempos();
        TotalMs = (int)(nTempos > 0 ? tempos[nTempos - 1].cumulUs / 1000 : 0);
    }

    public static void InitMidi()
    {
        // Try KDMAPI (OmniMIDI) first
        try {
            IntPtr h = LoadLibrary("OmniMIDI.dll");
            if (h != IntPtr.Zero) {
                IntPtr initFn = GetProcAddress(h, "InitializeKDMAPIStream");
                IntPtr sendFn = GetProcAddress(h, "SendDirectData");
                IntPtr termFn = GetProcAddress(h, "TerminateKDMAPIStream");
                if (initFn != IntPtr.Zero && sendFn != IntPtr.Zero) {
                    var init = (KDMAPI_Init)Marshal.GetDelegateForFunctionPointer(initFn, typeof(KDMAPI_Init));
                    if (init() != 0) {
                        kdmapiSend = (KDMAPI_SendDirect)Marshal.GetDelegateForFunctionPointer(sendFn, typeof(KDMAPI_SendDirect));
                        useKdmapi = true;
                        Console.WriteLine("KDMAPI initialized");
                    }
                }
            }
        } catch { }
        if (!useKdmapi) {
            int r = midiOutOpen(out midiH, -1, IntPtr.Zero, IntPtr.Zero, 0);
            if (r != 0) throw new Exception("Failed to open MIDI device");
            Console.WriteLine("WinMM MIDI opened");
        }
        InitTracks();
    }

    public static void StartPlayback()
    {
        Console.Error.WriteLine("StartPlayback: priming vis...");
        playing = true;
        startQpc = Stopwatch.GetTimestamp();
        // Prime visPos: process initial visual catch-up without event cap
        int saved = visEventsThisFrame;
        visEventsThisFrame = -99999999;
        long primingStart = Stopwatch.GetTimestamp();
        curTick = 0;
        int micros = GetTempoMicros();
        if (micros < 50000) micros = 50000;
        curLookaheadTicks = (int)(LOOKAHEAD_TICKS * Math.Pow(500000.0 / micros, 0.7));
        if (curLookaheadTicks < 60) curLookaheadTicks = 60;
        int primeLimit = curLookaheadTicks * 2;
        for (int t = 0; t < nTracks; t++)
            AdvanceTrack(t, primeLimit, false);
        long primingUs = (Stopwatch.GetTimestamp() - primingStart) * 1000000 / Stopwatch.Frequency;
        Console.Error.WriteLine("StartPlayback: priming done in " + (primingUs/1000) + "ms, visN=" + visN);
        visEventsThisFrame = saved;
    }

    static long frameTimeMax;

    public static void Frame()
    {
        if (!playing) return;
        try {
        long frameStartTicks = Stopwatch.GetTimestamp();
        int now = (int)((frameStartTicks - startQpc) * 1000.0 / Stopwatch.Frequency);
        if (now < 0) now = 0;
        curMs = now;
        curTick = MsToTick(now);
        int micros = GetTempoMicros();
        if (micros < 50000) micros = 50000;
        curLookaheadTicks = (int)(LOOKAHEAD_TICKS * Math.Pow(500000.0 / micros, 0.7));
        if (curLookaheadTicks < 60) curLookaheadTicks = 60;

        int visTargetTick = curTick + curLookaheadTicks * 2;
        int pastTick = Math.Max(curTick - curLookaheadTicks, 0);

        // Detect freeze: if frame > 3s, abort
        long sinceLast = (now - lastFrameMs);
        if (sinceLast > 3000 && lastFrameMs > 0)
        {
            Console.Error.WriteLine("FREEZE DETECTED: " + sinceLast + "ms since last frame, stage=" + dbgStage);
            Console.Error.WriteLine("  curTick=" + curTick + " activeTracks=" + CountActive());
        }
        lastFrameMs = now;
        dbgStage = "frame_start";

        midiEventsThisFrame = 0;
        dbgStage = "audio";
        long t0 = Stopwatch.GetTimestamp();
        for (int t = 0; t < nTracks; t++)
            AdvanceTrack(t, curTick, true);
        long t1 = Stopwatch.GetTimestamp();
        long audioUs = (t1 - t0) * 1000000 / Stopwatch.Frequency;

        visEventsThisFrame = 0;
        dbgStage = "visual";
        long t1b = Stopwatch.GetTimestamp();
        int startVisTrack = (int)((frameStartTicks * 2654435761L) % nTracks);
        if (startVisTrack < 0) startVisTrack = 0;
        for (int i = 0; i < nTracks; i++)
        {
            int t = (startVisTrack + i) % nTracks;
            AdvanceTrack(t, visTargetTick, false);
        }
        long t2 = Stopwatch.GetTimestamp();
        long visUs = (t2 - t1b) * 1000000 / Stopwatch.Frequency;

        dbgStage = "compact";

        // Compact vis[] only when near capacity (spares per-frame dict rebuild)
        if (visN >= MAX_VIS - 50000)
        {
            int wi = 0;
            for (int i = 0; i < visN; i++)
            {
                var n = vis[i];
                bool dead = (n.done && n.endTick < pastTick);
                if (!dead) { compactRemap[i] = wi; vis[wi++] = n; }
                else compactRemap[i] = -1;
            }
            var newMap = new Dictionary<int, int>();
            foreach (var kv in activeNoteMap)
            {
                int ni = compactRemap[kv.Value];
                if (ni >= 0) newMap[kv.Key] = ni;
            }
            activeNoteMap = newMap;
            visN = wi;
        }
        long t3 = Stopwatch.GetTimestamp();

        long tFrameEnd = Stopwatch.GetTimestamp();
        long dt = tFrameEnd - frameStartTicks;
        if (dt > frameTimeMax) { frameTimeMax = dt; }
        if ((now % 500) < 16) {
            int aMs = (int)((t1 - t0) * 1000 / Stopwatch.Frequency);
            int vMs = (int)((t2 - t1b) * 1000 / Stopwatch.Frequency);
            int cMs = (int)((tFrameEnd - t2) * 1000 / Stopwatch.Frequency);
            int maxMs = (int)(frameTimeMax * 1000 / Stopwatch.Frequency);
            Console.WriteLine("now=" + now + "midiEv=" + midiEventsThisFrame + "visEv=" + visEventsThisFrame + "visN=" + visN + "a=" + aMs + "v=" + vMs + "c=" + cMs + "ms max=" + maxMs);
        }

        dbgStage = "done";

        bool any = false;
        for (int i = 0; i < nTracks; i++)
            if (tracks[i].active) { any = true; break; }
        if (!any) playing = false;
        } catch (Exception ex) { lastError = ex.Message + "\n" + ex.StackTrace; error = lastError; playing = false; }
    }

    public static void Stop()
    {
        playing = false;
        for (int c = 0; c < 16; c++)
            MidiSend((uint)(0xB0 | c | (0x7B << 8) | (0 << 16)));
        if (!useKdmapi) midiOutClose(midiH);
    }

    static void ReadHeader()
    {
        if (data.Length < 14)
            throw new Exception("File too small to be a valid MIDI file");
        if (data[0] != 'M' || data[1] != 'T' || data[2] != 'h' || data[3] != 'd')
            throw new Exception("Not a MIDI file");
        nTracks = BE16(10);
        division = BE16(12);
        if (division < 0) division = -division;
    }

    static int BE16(int p) { return p + 1 < data.Length ? (data[p] << 8) | data[p + 1] : 0; }
    static int BE32(int p) { return p + 3 < data.Length ? (data[p] << 24) | (data[p + 1] << 16) | (data[p + 2] << 8) | data[p + 3] : 0; }

    static int VLQ(ref int p)
    {
        int v = 0; byte b;
        do {
            if (p >= data.Length) return 0;
            b = data[p++]; v = (v << 7) | (b & 0x7F);
        } while ((b & 0x80) != 0);
        return v;
    }

    static void PrescanTempos()
    {
        tempos = new Tempo[128];
        nTempos = 0;
        int pos = 14;
        for (int t = 0; t < nTracks; t++)
        {
            pos += 4; int sz = BE32(pos); pos += 4; int end = pos + sz;
            if (end > data.Length) end = data.Length;
            int p = pos; byte st = 0; long tk = 0;
            while (p < end && p < data.Length)
            {
                if (p + 4 > data.Length) break;
                tk += VLQ(ref p);
                if (p >= end || p >= data.Length) break;
                byte ev = data[p++];
                if (ev == 0xFF)
                {
                    int mt = data[p++]; int len = VLQ(ref p);
                    if (p > end || p > data.Length) break;
                    if (mt == 0x51) {
                        int us = (data[p] << 16) | (data[p + 1] << 8) | data[p + 2];
                        if (nTempos >= tempos.Length) Array.Resize(ref tempos, tempos.Length * 2);
                        tempos[nTempos++] = new Tempo { tick = tk, micros = us };
                    }
                    p += len;
                }
                else if (ev >= 0x80 && ev <= 0xEF) { st = ev; p += (st >= 0xC0 && st <= 0xDF) ? 1 : 2; }
                else if (ev >= 0xF0 && ev <= 0xF7) { if (ev == 0xF0) { int len = VLQ(ref p); p += len; } }
            }
            pos = end;
        }
        Array.Sort(tempos, 0, nTempos, new TempoComparer());
        int wi = 0;
        for (int i = 0; i < nTempos; i++)
        {
            if (wi > 0 && tempos[i].tick == tempos[wi - 1].tick)
                tempos[wi - 1] = tempos[i];
            else
                tempos[wi++] = tempos[i];
        }
        nTempos = wi;
        long cumul = 0;
        for (int i = 0; i < nTempos; i++)
        {
            long prevTick = i > 0 ? tempos[i - 1].tick : 0;
            int prevUs = i > 0 ? tempos[i - 1].micros : 500000;
            cumul += (tempos[i].tick - prevTick) * (long)prevUs / division;
            tempos[i].cumulUs = cumul;
        }
    }

    static int MsToTick(int ms)
    {
        long targetUs = (long)ms * 1000;
        if (nTempos == 0) return (int)(targetUs * division / 500000);
        int lo = 0, hi = nTempos;
        while (lo < hi) { int m = (lo + hi) / 2; if (tempos[m].cumulUs <= targetUs) lo = m + 1; else hi = m; }
        int idx = lo - 1;
        if (idx < 0) return (int)(targetUs * division / 500000);
        long extraUs = targetUs - tempos[idx].cumulUs;
        if (extraUs < 0) extraUs = 0;
        return (int)(tempos[idx].tick + extraUs * division / Math.Max(tempos[idx].micros, 1));
    }

    static void InitTracks()
    {
        tracks = new TState[nTracks];
        int pos = 14;
        for (int t = 0; t < nTracks; t++)
        {
            pos += 4; int sz = BE32(pos); pos += 4;
            int trackEnd = pos + sz;
            if (trackEnd > data.Length) trackEnd = data.Length;
            tracks[t] = new TState {
                audPos = pos, visPos = pos, end = trackEnd,
                audTick = 0, visTick = 0, audStatus = 0, visStatus = 0, active = true
            };
            pos = tracks[t].end;
        }
    }

    static int CountActive()
    {
        int c = 0;
        for (int i = 0; i < nTracks; i++)
            if (tracks[i].active) c++;
        return c;
    }

    static int GetTempoMicros()
    {
        if (nTempos == 0) return 500000;
        int lo = 0, hi = nTempos;
        while (lo < hi) { int m = (lo + hi) / 2; if (tempos[m].tick <= curTick) lo = m + 1; else hi = m; }
        int idx = lo - 1;
        if (idx < 0) return 500000;
        return tempos[idx].micros;
    }

    static void AdvanceTrack(int t, int limitTick, bool audio)
    {
        if (!tracks[t].active) return;
        int p = audio ? tracks[t].audPos : tracks[t].visPos;
        int end = tracks[t].end;
        long tk = audio ? tracks[t].audTick : tracks[t].visTick;
        byte st = audio ? tracks[t].audStatus : tracks[t].visStatus;
        int cutoffTick = audio ? 0 : curTick - curLookaheadTicks;

        while (p < end && p < data.Length)
        {
            int savedP = p;
            if (p + 4 > data.Length) { tracks[t].active = false; break; }
            long delta = VLQ(ref p);
            if (p >= end || p >= data.Length) { tracks[t].active = false; break; }
            tk += delta;
            if (tk > limitTick) { p = savedP; tk -= delta; break; }

            byte ev = data[p++];
            if (ev == 0xFF)
            {
                int mt = data[p++]; int len = VLQ(ref p);
                if (p > end || p > data.Length) { tracks[t].active = false; break; }
                if (mt == 0x2F) { tracks[t].active = false; break; }
                p += len;
                if (p > end) p = end;
            }
            else if (ev >= 0x80 && ev <= 0xEF)
            {
                int cmd = ev & 0xF0, ch = ev & 0x0F;
                int d1 = data[p++];
                int d2 = (cmd >= 0xC0 && cmd <= 0xDF) ? 0 : data[p++];
                st = (byte)ev;
                if (audio) {
                    if (++midiEventsThisFrame > 50000) { p = savedP; tk -= delta; break; }
                    MidiSend((uint)(ev | (d1 << 8) | (d2 << 16)));
                }
                else {
                    if (tk < cutoffTick) {
                        if (cmd == 0x80 || (cmd == 0x90 && d2 == 0)) {
                            int id = (d1 << 8) | ch;
                            int idx;
                            if (activeNoteMap.TryGetValue(id, out idx) && idx < visN && !vis[idx].done) {
                                vis[idx].done = true;
                                vis[idx].endTick = (int)tk;
                                activeNoteMap.Remove(id);
                            }
                        }
                    } else {
                        if (visEventsThisFrame >= VIS_CAP) { p = savedP; tk -= delta; break; }
                        visEventsThisFrame++;
                        AddNote(t, cmd, ch, d1, d2, (int)tk);
                    }
                }
            }
            else if (ev >= 0xF0 && ev <= 0xF7)
            {
                if (ev == 0xF0) {
                    int len = VLQ(ref p);
                    if (p > end || p > data.Length) { tracks[t].active = false; break; }
                    p += len;
                }
            }
            else
            {
                int cmd = st & 0xF0, ch = st & 0x0F;
                int d1 = ev;
                int d2 = p < end ? data[p++] : 0;
                if (audio) {
                    if (++midiEventsThisFrame > 50000) { p = savedP; tk -= delta; break; }
                    MidiSend((uint)(st | (d1 << 8) | (d2 << 16)));
                }
                else {
                    if (tk < cutoffTick) {
                        if (cmd == 0x80 || (cmd == 0x90 && d2 == 0)) {
                            int id = (d1 << 8) | ch;
                            int idx;
                            if (activeNoteMap.TryGetValue(id, out idx) && idx < visN && !vis[idx].done) {
                                vis[idx].done = true;
                                vis[idx].endTick = (int)tk;
                                activeNoteMap.Remove(id);
                            }
                        }
                    } else {
                        if (visEventsThisFrame >= VIS_CAP) { p = savedP; tk -= delta; break; }
                        visEventsThisFrame++;
                        AddNote(t, cmd, ch, d1, d2, (int)tk);
                    }
                }
            }
        }

        if (p >= end || p >= data.Length) tracks[t].active = false;

        if (audio) { tracks[t].audPos = p; tracks[t].audTick = tk; tracks[t].audStatus = st; }
        else { tracks[t].visPos = p; tracks[t].visTick = tk; tracks[t].visStatus = st; }
    }

    public static void ToggleTrackColor() { useTrackColor = !useTrackColor; }

    static void AddNote(int track, int cmd, int ch, int d1, int d2, int tick)
    {
        int id = (d1 << 8) | ch;
        if (cmd == 0x90 && d2 > 0)
        {
            if (visN < MAX_VIS)
            {
                int idx = visN++;
                vis[idx] = new Note { startTick = tick, key = d1, vel = d2, chan = ch, track = track, done = false };
                activeNoteMap[id] = idx;
            }
        }
        else if (cmd == 0x80 || (cmd == 0x90 && d2 == 0))
        {
            int idx;
            if (activeNoteMap.TryGetValue(id, out idx) && idx < visN && !vis[idx].done)
            {
                vis[idx].done = true;
                vis[idx].endTick = tick;
                activeNoteMap.Remove(id);
            }
        }
    }

    static class Stopwatch
    {
        public static long GetTimestamp() { long c; QueryPerformanceCounter(out c); return c; }
        public static readonly long Frequency = GetFreq();
        static long GetFreq() { long f; QueryPerformanceFrequency(out f); return f; }
    }
}

public class MidiForm : Form {
    public MidiForm() { DoubleBuffered = true; }

    [STAThread]
    public static void Main(string[] args)
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);

        string midiFile = "";
        bool bench = false;
        foreach (string arg in args)
        {
            if (arg == "--bench") bench = true;
            else if (midiFile == "") midiFile = arg;
        }
        if (midiFile == "")
        {
            Console.Error.WriteLine("Usage: midiviz <file.mid> [--bench]");
            Environment.Exit(1);
        }

        MidiEngine.bench = bench;
        MidiEngine.Load(midiFile);
        if (bench)
        {
            Console.WriteLine("dur=" + MidiEngine.TotalMs + "ms (bench)");
            return;
        }

        Console.WriteLine("Loading... " + MidiEngine.TotalMs + "ms duration");

        MidiForm form = new MidiForm();
        form.Text = "MIDI Stream - " + System.IO.Path.GetFileName(midiFile);
        form.ClientSize = new System.Drawing.Size(1200, 600);
        form.StartPosition = FormStartPosition.CenterScreen;

        form.Paint += (s, e) => MidiRender.Paint(e.Graphics, form.ClientSize.Width, form.ClientSize.Height);

        form.KeyDown += (s, e) => {
            if (e.KeyCode == Keys.Escape) form.Close();
            else if (e.KeyCode == Keys.C) MidiEngine.ToggleTrackColor();
        };

        form.FormClosing += (s, e) => { MidiEngine.Playing = false; };

        MidiEngine.InitMidi();
        MidiEngine.StartPlayback();

        System.Windows.Forms.Timer timer = new System.Windows.Forms.Timer();
        timer.Interval = 16;
        timer.Tick += (s, e) => {
            if (MidiEngine.Playing)
            {
                MidiEngine.Frame();
                form.Invalidate();
            }
            string err = MidiEngine.error;
            if (err != null && err != "")
            {
                Console.Error.WriteLine("ERROR: " + err);
                MidiEngine.error = "";
            }
            if (!MidiEngine.Playing) timer.Stop();
        };
        timer.Start();

        Application.Run(form);

        timer.Stop();
        MidiEngine.Stop();
        Console.WriteLine("Done");
    }
}

public static class MidiRender
{
    static readonly Color[] ChanColors = new Color[] {
        Color.FromArgb(255,80,80), Color.FromArgb(255,160,80), Color.FromArgb(240,240,80),
        Color.FromArgb(80,255,80), Color.FromArgb(80,255,200), Color.FromArgb(80,180,255),
        Color.FromArgb(160,120,255), Color.FromArgb(220,100,255), Color.FromArgb(255,100,180),
        Color.FromArgb(255,140,100), Color.FromArgb(200,200,100), Color.FromArgb(100,220,220),
        Color.FromArgb(180,180,180), Color.FromArgb(100,100,255), Color.FromArgb(255,200,100),
        Color.FromArgb(200,100,200)
    };
    const int NOTE_W = 12;

    public static void Paint(Graphics g, int w, int h)
    {
        int kbH = 24;
        int nowTick = MidiEngine.curTick;
        int range = h - kbH;
        int lookaheadTicks = MidiEngine.curLookaheadTicks;
        if (lookaheadTicks < 60) lookaheadTicks = 480;

        g.Clear(Color.Black);

        if (MidiEngine.lastError != null)
        {
            using (var fb = new SolidBrush(Color.FromArgb(200, Color.Black)))
                g.FillRectangle(fb, 0, 0, w, 60);
            using (var fp = new SolidBrush(Color.Red))
                g.DrawString(MidiEngine.lastError, SystemFonts.DefaultFont, fp, 4, 4);
        }


        // Piano keys - full MIDI range 0-127
        g.FillRectangle(Brushes.Black, 0, h - kbH, 128 * NOTE_W, kbH);
        for (int i = 0; i < 128; i++)
        {
            int r = i % 12;
            bool white = (r != 1 && r != 3 && r != 6 && r != 8 && r != 10);
            if (white)
            {
                int x = i * NOTE_W;
                g.FillRectangle(Brushes.White, x, h - kbH, NOTE_W - 1, kbH);
                g.DrawRectangle(Pens.DimGray, x, h - kbH, NOTE_W - 1, kbH);
            }
        }
        int bw = NOTE_W * 2 / 3;
        int bh = kbH * 2 / 3;
        int by = h - bh;
        for (int i = 0; i < 128; i++)
        {
            int r = i % 12;
            bool black = (r == 1 || r == 3 || r == 6 || r == 8 || r == 10);
            if (black)
            {
                int x = i * NOTE_W - bw / 2;
                g.FillRectangle(Brushes.Black, x, by, bw, bh);
                g.DrawRectangle(Pens.DimGray, x, by, bw, bh);
            }
        }

        int nV = MidiEngine.visN;
        var vis = MidiEngine.VisArr;
        int startIdx = 0;
        while (startIdx < nV)
        {
            var n = vis[startIdx];
            if (n.done && n.endTick < nowTick - lookaheadTicks) { startIdx++; continue; }
            break;
        }

        for (int i = startIdx; i < nV; i++)
        {
            var n = vis[i];
            if (n.startTick >= nowTick + lookaheadTicks) break;

            int x = n.key * NOTE_W;
            int bot = h - kbH;
            int startY = bot - (n.startTick - nowTick) * range / lookaheadTicks;
            if (startY < 0) continue;

            int endY;
            int blockH = 20;
            if (n.done)
            {
                endY = bot - (n.endTick - nowTick) * range / lookaheadTicks;
                if (endY > h) continue;
            }
            else
            {
                endY = startY - blockH;
                if (endY > h) continue;
            }
            if (startY <= endY) continue;

            int hh = startY - endY;
            if (hh < blockH) hh = blockH;
            if (hh > range) hh = range;
            int y = startY - hh;

            Color col = MidiEngine.useTrackColor ? ChanColors[n.track & 0xF] : ChanColors[n.chan & 0xF];
            int alpha = n.done ? 160 : 220;
            using (var b = new SolidBrush(Color.FromArgb(alpha, col)))
                g.FillRectangle(b, x, y, NOTE_W - 1, hh);
        }
    }
}
