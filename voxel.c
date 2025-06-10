/**
 * voxel: High-performance video player for the modern terminal.
 *
 * This version uses the libav* (ffmpeg) and PulseAudio APIs directly for
 * efficient, multithreaded decoding and playback.
 *
 * Architecture:
 * - Demux/Decode Thread: Reads from the video file, decodes video frames,
 *   scales them to the target size, and places the final RGB buffer into a "full" queue.
 * - Audio Thread: Decodes audio packets, synchronizes them against a master system clock,
 *   and plays them using PulseAudio.
 * - Main (Render) Thread: Takes RGB buffers from the "full" queue, synchronizes them
 *   against the master clock, renders them to the terminal, and returns the used
 *   buffer to an "empty" queue for reuse. This thread also implements a frame-dropping
 *   mechanism to maintain sync if rendering falls behind.
 *
 * This producer-consumer model with a buffer pool minimizes memory allocations and
 * allows decoding/scaling to happen in parallel with rendering, maximizing performance.
 */

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/resource.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>

// FFmpeg (libav) libraries
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
#include <libavutil/time.h>
#include <libavutil/opt.h>

// Audio library
#include <pulse/simple.h>
#include <pulse/error.h>

// =====================================================================================
// == Constants and Global Configuration
// =====================================================================================

#define OUTPUT_BUFFER_SIZE (1024 * 256) // 256 KB buffer for terminal output
#define MAX_RENDER_TIMES 8192           // For statistics: max frames to record timings for

typedef enum {
    MODE_FULL,
    MODE_HALF,
    MODE_QUADRANT
} RenderMode;

// For 2x2 quadrant mode. Index is bitmask: 8=BR, 4=BL, 2=TR, 1=TL
static const char* QUADRANT_CHARS[] = {
    " ", "▘", "▝", "▀", "▖", "▌", "▞", "▛",
    "▗", "▚", "▐", "▟", "▄", "▙", "▜", "█"
};
// Helper for popcount on a 4-bit number
static const int BITS_SET[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

static volatile sig_atomic_t g_interrupted = 0;

// =====================================================================================
// == Terminal I/O and State
// =====================================================================================

typedef struct {
    uint8_t fg[3];
    uint8_t bg[3];
    bool fg_is_set;
    bool bg_is_set;
} TerminalState;

typedef struct {
    char buffer[OUTPUT_BUFFER_SIZE];
    size_t buffer_pos;
    long bytes_written;
    int cursor_row, cursor_col;
    TerminalState current_state;
} TerminalContext;

void buffer_flush(TerminalContext *term) {
    if (term->buffer_pos > 0) {
        fwrite(term->buffer, 1, term->buffer_pos, stdout);
        term->bytes_written += term->buffer_pos;
        term->buffer_pos = 0;
    }
}

void buffer_append(TerminalContext *term, const char *format, ...) {
    if (term->buffer_pos >= OUTPUT_BUFFER_SIZE - 256) {
        buffer_flush(term);
    }
    va_list args;
    va_start(args, format);
    int bytes = vsnprintf(term->buffer + term->buffer_pos, OUTPUT_BUFFER_SIZE - term->buffer_pos, format, args);
    va_end(args);
    if (bytes > 0) {
        term->buffer_pos += bytes;
    }
}

void buffer_append_run(TerminalContext *term, const char* s, int count) {
    if (count <= 0) return;

    size_t len = strlen(s);
    if (len == 0) return;

    if (len * count >= OUTPUT_BUFFER_SIZE) {
        for (int i = 0; i < count; ++i) {
            buffer_append(term, "%s", s);
        }
        return;
    }

    if (term->buffer_pos + (len * count) >= OUTPUT_BUFFER_SIZE) {
        buffer_flush(term);
    }

    char* dest = term->buffer + term->buffer_pos;
    if (len == 1) {
        memset(dest, s[0], count);
    } else {
        for (int i = 0; i < count; i++) {
            memcpy(dest, s, len);
            dest += len;
        }
    }
    term->buffer_pos += len * count;
}

// =====================================================================================
// == Terminal Rendering Logic
// =====================================================================================

static inline float color_dist_sq(const uint8_t c1[3], const uint8_t c2[3]) {
    long dr = c1[0] - c2[0], dg = c1[1] - c2[1], db = c1[2] - c2[2];
    float rm = (c1[0] + c2[0]) / 512.0f;
    return ((2.0f + rm) * dr * dr + 4.0f * dg * dg + (3.0f - rm) * db * db);
}

static inline bool are_colors_similar(const uint8_t c1[3], const uint8_t c2[3], int render_tolerance_sq) {
    if (render_tolerance_sq == 0) {
        return memcmp(c1, c2, 3) == 0;
    }
    return color_dist_sq(c1, c2) < render_tolerance_sq;
}

static inline int count_digits(int n) {
    if (n < 0) n = -n;
    if (n < 10) return 1;
    if (n < 100) return 2;
    if (n < 1000) return 3;
    if (n < 10000) return 4;
    return 5;
}

static void move_cursor(TerminalContext *term, int target_row, int target_col) {
    if (term->cursor_row == target_row && term->cursor_col == target_col) return;

    if (term->cursor_col != 0 && target_col == 1 && target_row == term->cursor_row + 1) {
        buffer_append(term, "\n");
        term->cursor_row = target_row;
        term->cursor_col = target_col;
        return;
    }

    int abs_cost = 4 + count_digits(target_row) + count_digits(target_col);
    int rel_cost = 0;
    int row_diff = target_row - term->cursor_row;
    int col_diff = target_col - term->cursor_col;
    if (row_diff != 0) rel_cost += 3 + count_digits(row_diff);
    if (col_diff != 0) rel_cost += 3 + count_digits(col_diff);

    if (term->cursor_row == 0 || abs_cost <= rel_cost) {
        buffer_append(term, "\x1b[%d;%dH", target_row, target_col);
    } else {
        if (row_diff > 0) buffer_append(term, "\x1b[%dB", row_diff); else if (row_diff < 0) buffer_append(term, "\x1b[%dA", -row_diff);
        if (col_diff > 0) buffer_append(term, "\x1b[%dC", col_diff); else if (col_diff < 0) buffer_append(term, "\x1b[%dD", -col_diff);
    }
    term->cursor_row = target_row;
    term->cursor_col = target_col;
}

void set_colors(TerminalContext *term, const uint8_t* new_fg, const uint8_t* new_bg, int render_tolerance_sq) {
    bool fg_changed = !term->current_state.fg_is_set || !are_colors_similar(term->current_state.fg, new_fg, render_tolerance_sq);
    bool bg_changed = !term->current_state.bg_is_set || !are_colors_similar(term->current_state.bg, new_bg, render_tolerance_sq);

    if (fg_changed && bg_changed) {
        buffer_append(term, "\x1b[38;2;%d;%d;%d;48;2;%d;%d;%dm", new_fg[0], new_fg[1], new_fg[2], new_bg[0], new_bg[1], new_bg[2]);
        memcpy(term->current_state.fg, new_fg, 3);
        memcpy(term->current_state.bg, new_bg, 3);
        term->current_state.fg_is_set = term->current_state.bg_is_set = true;
    } else if (fg_changed) {
        buffer_append(term, "\x1b[38;2;%d;%d;%dm", new_fg[0], new_fg[1], new_fg[2]);
        memcpy(term->current_state.fg, new_fg, 3);
        term->current_state.fg_is_set = true;
    } else if (bg_changed) {
        buffer_append(term, "\x1b[48;2;%d;%d;%dm", new_bg[0], new_bg[1], new_bg[2]);
        memcpy(term->current_state.bg, new_bg, 3);
        term->current_state.bg_is_set = true;
    }
}

void render_frame_full_block(TerminalContext *term, int w, int h, const uint8_t* current_rgb, const uint8_t* prev_rgb, long frame, int render_tolerance_sq) {
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ) {
            const uint8_t* cur_p = &current_rgb[(y * w + x) * 3];
            if (frame > 0 && are_colors_similar(cur_p, &prev_rgb[(y * w + x) * 3], render_tolerance_sq)) {
                x++;
                continue;
            }

            move_cursor(term, y + 1, x + 1);
            if (!term->current_state.bg_is_set || !are_colors_similar(term->current_state.bg, cur_p, render_tolerance_sq)) {
                buffer_append(term, "\x1b[48;2;%d;%d;%dm", cur_p[0], cur_p[1], cur_p[2]);
                memcpy(term->current_state.bg, cur_p, 3);
                term->current_state.bg_is_set = true;
                term->current_state.fg_is_set = false;
            }

            int run_len = 1;
            for (int i = x + 1; i < w; ++i) {
                if (!are_colors_similar(cur_p, &current_rgb[(y * w + i) * 3], render_tolerance_sq)) break;
                run_len++;
            }
            buffer_append_run(term, " ", run_len);
            term->cursor_col += run_len;
            x += run_len;
        }
    }
}

void render_frame_half_block(TerminalContext *term, int w, int h, const uint8_t* rgb, const uint8_t* prev_rgb, long frame, int render_tolerance_sq) {
    int char_h = h / 2;
    for (int y = 0; y < char_h; ++y) {
        for (int x = 0; x < w; ) {
            const uint8_t* top_p = &rgb[((y * 2) * w + x) * 3];
            const uint8_t* bot_p = &rgb[((y * 2 + 1) * w + x) * 3];
            if (frame > 0 && are_colors_similar(top_p, &prev_rgb[((y * 2) * w + x) * 3], render_tolerance_sq) && are_colors_similar(bot_p, &prev_rgb[((y * 2 + 1) * w + x) * 3], render_tolerance_sq)) {
                x++;
                continue;
            }

            move_cursor(term, y + 1, x + 1);
            int cost_normal = (!term->current_state.fg_is_set || !are_colors_similar(term->current_state.fg, top_p, render_tolerance_sq)) + (!term->current_state.bg_is_set || !are_colors_similar(term->current_state.bg, bot_p, render_tolerance_sq));
            int cost_flipped = (!term->current_state.fg_is_set || !are_colors_similar(term->current_state.fg, bot_p, render_tolerance_sq)) + (!term->current_state.bg_is_set || !are_colors_similar(term->current_state.bg, top_p, render_tolerance_sq));

            const char* char_to_print;
            const uint8_t *new_fg, *new_bg;
            if (cost_normal <= cost_flipped) {
                char_to_print = "\xE2\x96\x80"; // U+2580 Upper half block
                new_fg = top_p;
                new_bg = bot_p;
            } else {
                char_to_print = "\xE2\x96\x84"; // U+2584 Lower half block
                new_fg = bot_p;
                new_bg = top_p;
            }
            set_colors(term, new_fg, new_bg, render_tolerance_sq);

            int run_len = 1;
            for (int i = x + 1; i < w; ++i) {
                if (!are_colors_similar(top_p, &rgb[((y * 2) * w + i) * 3], render_tolerance_sq) || !are_colors_similar(bot_p, &rgb[((y * 2 + 1) * w + i) * 3], render_tolerance_sq)) break;
                run_len++;
            }
            buffer_append_run(term, char_to_print, run_len);
            term->cursor_col += run_len;
            x += run_len;
        }
    }
}

typedef struct {
    uint8_t fg[3], bg[3];
    int mask;
} QuadrantCell;

void get_optimal_quadrant(const uint8_t* px[4], QuadrantCell* cell) {
    float best_error = (float)LONG_MAX;
    for (int i = 1; i < (1 << 3); ++i) {
        long c1_sum[3] = {0}, c2_sum[3] = {0};
        int c1_count = 0, c2_count = 0;
        int current_mask = 0;
        for (int j = 0; j < 4; ++j) {
            if ((i >> j) & 1) {
                c1_sum[0] += px[j][0]; c1_sum[1] += px[j][1]; c1_sum[2] += px[j][2];
                c1_count++;
            } else {
                c2_sum[0] += px[j][0]; c2_sum[1] += px[j][1]; c2_sum[2] += px[j][2];
                c2_count++;
            }
        }
        if (c2_count == 0) continue;

        uint8_t c1_avg[3] = {c1_sum[0] / c1_count, c1_sum[1] / c1_count, c1_sum[2] / c1_count};
        uint8_t c2_avg[3] = {c2_sum[0] / c2_count, c2_sum[1] / c2_count, c2_sum[2] / c2_count};

        float current_error = 0;
        for (int j = 0; j < 4; ++j) {
            if ((i >> j) & 1) {
                current_error += color_dist_sq(px[j], c1_avg);
                current_mask |= (1 << j);
            } else {
                current_error += color_dist_sq(px[j], c2_avg);
            }
        }
        if (current_error < best_error) {
            best_error = current_error;
            cell->mask = current_mask;
            memcpy(cell->fg, c1_avg, 3);
            memcpy(cell->bg, c2_avg, 3);
        }
    }
    // Invert mask if it has more than 2 bits set, saving a color change
    if (BITS_SET[cell->mask] > 2) {
        cell->mask = 0xF & ~cell->mask;
        uint8_t temp[3];
        memcpy(temp, cell->fg, 3);
        memcpy(cell->fg, cell->bg, 3);
        memcpy(cell->bg, temp, 3);
    }
}

void render_frame_quadrant(TerminalContext *term, int w, int h, const uint8_t* rgb, const uint8_t* prev_rgb, long frame, int render_tolerance_sq) {
    int char_w = w / 2, char_h = h / 2;

    QuadrantCell *cell_row = malloc(char_w * sizeof(QuadrantCell));
    if (!cell_row) return;

    for (int y = 0; y < char_h; ++y) {
        bool row_has_changes = false;
        for (int x = 0; x < char_w; ++x) {
            const uint8_t* px_ptr[4] = {
                &rgb[((y * 2) * w + x * 2) * 3],
                &rgb[((y * 2) * w + x * 2 + 1) * 3],
                &rgb[((y * 2 + 1) * w + x * 2) * 3],
                &rgb[((y * 2 + 1) * w + x * 2 + 1) * 3]
            };
            if (frame > 0 &&
                are_colors_similar(px_ptr[0], &prev_rgb[((y * 2) * w + x * 2) * 3], render_tolerance_sq) &&
                are_colors_similar(px_ptr[1], &prev_rgb[((y * 2) * w + x * 2 + 1) * 3], render_tolerance_sq) &&
                are_colors_similar(px_ptr[2], &prev_rgb[((y * 2 + 1) * w + x * 2) * 3], render_tolerance_sq) &&
                are_colors_similar(px_ptr[3], &prev_rgb[((y * 2 + 1) * w + x * 2 + 1) * 3], render_tolerance_sq))
            {
                cell_row[x].mask = -1;
                continue;
            }

            row_has_changes = true;
            get_optimal_quadrant(px_ptr, &cell_row[x]);
        }

        if (!row_has_changes) {
            continue;
        }

        for (int x = 0; x < char_w; ) {
            if (cell_row[x].mask == -1) {
                x++;
                continue;
            }

            move_cursor(term, y + 1, x + 1);

            set_colors(term, cell_row[x].fg, cell_row[x].bg, render_tolerance_sq);
            const char* char_to_print = QUADRANT_CHARS[cell_row[x].mask];

            int run_len = 1;
            for (int i = x + 1; i < char_w; ++i) {
                if (cell_row[i].mask != cell_row[x].mask ||
                    !are_colors_similar(cell_row[i].fg, cell_row[x].fg, render_tolerance_sq) ||
                    !are_colors_similar(cell_row[i].bg, cell_row[x].bg, render_tolerance_sq)) {
                    break;
                }
                run_len++;
            }

            buffer_append_run(term, char_to_print, run_len);
            term->cursor_col += run_len;
            x += run_len;
        }
    }

    free(cell_row);
}

// =====================================================================================
// == Threading Primitives (Generic Queue)
// =====================================================================================

#define QUEUE_CAPACITY 60
typedef struct {
    void* items[QUEUE_CAPACITY];
    int size;
    int read_idx, write_idx;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    bool finished;
} GenericQueue;

typedef struct {
    uint8_t* rgb_data;
    double pts; // Presentation timestamp in seconds
} QueuedFrame;

void queue_init(GenericQueue* q) {
    memset(q, 0, sizeof(GenericQueue));
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->cond, NULL);
}

int queue_put(GenericQueue* q, void* item) {
    pthread_mutex_lock(&q->mutex);
    while (q->size >= QUEUE_CAPACITY && !q->finished) {
        pthread_cond_wait(&q->cond, &q->mutex);
    }
    if (q->finished) {
        pthread_mutex_unlock(&q->mutex);
        return -1; // Indicate queue is closed
    }
    q->items[q->write_idx] = item;
    q->write_idx = (q->write_idx + 1) % QUEUE_CAPACITY;
    q->size++;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
    return 0;
}

int queue_get(GenericQueue* q, void** item) {
    pthread_mutex_lock(&q->mutex);
    while (q->size == 0 && !q->finished) {
        pthread_cond_wait(&q->cond, &q->mutex);
    }
    if (q->size == 0 && q->finished) {
        pthread_mutex_unlock(&q->mutex);
        return -1; // Indicate end of stream
    }
    *item = q->items[q->read_idx];
    q->read_idx = (q->read_idx + 1) % QUEUE_CAPACITY;
    q->size--;
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
    return 0;
}

// Signal that no more items will be put into the queue.
void queue_finish(GenericQueue* q) {
    pthread_mutex_lock(&q->mutex);
    q->finished = true;
    pthread_cond_broadcast(&q->cond); // Wake up all waiting threads
    pthread_mutex_unlock(&q->mutex);
}

void queue_destroy(GenericQueue* q, void (*free_func)(void*)) {
    if (free_func) {
        void* item;
        while (queue_get(q, &item) == 0) {
            free_func(item);
        }
    }
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond);
}

// =====================================================================================
// == Player State and Thread Functions
// =====================================================================================

typedef struct {
    AVFormatContext *format_ctx;
    int video_stream_idx;
    int audio_stream_idx;
    int render_w, render_h;

    // A/V sync
    volatile double audio_clock; // Progress of audio stream (for stats)
    volatile bool quit;
    pa_simple *pa_s;
    volatile double playback_start_time;

    // Queues for thread communication
    GenericQueue full_q;      // Populated QueuedFrame objects ready for rendering
    GenericQueue empty_q;     // RGB buffers ready to be filled by decoder
    GenericQueue frame_pool_q; // Empty QueuedFrame objects ready for reuse
    GenericQueue audio_q;     // Raw audio packets for the audio thread
} PlayerState;

static void sigint_handler(int signum) {
    (void)signum; // Unused
    g_interrupted = 1;
}

/**
 * The decode thread's job is to:
 * 1. Read packets (video or audio) from the media file.
 * 2. If it's an audio packet, push it to the audio queue.
 * 3. If it's a video packet, decode it to a raw AVFrame.
 * 4. Get an empty RGB buffer from the `empty_q`.
 * 5. Scale the AVFrame into the RGB buffer.
 * 6. Push the filled RGB buffer to the `full_q` for the main thread to render.
 */
void* decode_thread_func(void* arg) {
    PlayerState* s = (PlayerState*)arg;

    AVStream* v_stream = s->format_ctx->streams[s->video_stream_idx];
    AVCodecParameters* v_codec_params = v_stream->codecpar;
    const AVCodec* v_codec = avcodec_find_decoder(v_codec_params->codec_id);
    AVCodecContext* v_codec_ctx = avcodec_alloc_context3(v_codec);
    avcodec_parameters_to_context(v_codec_ctx, v_codec_params);
    if (avcodec_open2(v_codec_ctx, v_codec, NULL) < 0) { s->quit = true; return NULL; }

    struct SwsContext* sws_ctx = sws_getContext(v_codec_params->width, v_codec_params->height, v_codec_params->format,
        s->render_w, s->render_h, AV_PIX_FMT_RGB24, SWS_BICUBIC, NULL, NULL, NULL);
    if (!sws_ctx) { s->quit = true; avcodec_free_context(&v_codec_ctx); return NULL; }

    AVPacket* pkt = av_packet_alloc();
    AVFrame* dec_frame = av_frame_alloc();
    AVRational time_base = v_stream->time_base;

    while (!s->quit && !g_interrupted && av_read_frame(s->format_ctx, pkt) >= 0) {
        if (pkt->stream_index == s->video_stream_idx) {
            if (avcodec_send_packet(v_codec_ctx, pkt) == 0) {
                while (true) {
                    int ret = avcodec_receive_frame(v_codec_ctx, dec_frame);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
                    if (ret < 0) { s->quit = true; break; }

                    uint8_t *rgb_buffer = NULL;
                    if (queue_get(&s->empty_q, (void**)&rgb_buffer) != 0) break;

                    QueuedFrame* q_frame = NULL;
                    if (queue_get(&s->frame_pool_q, (void**)&q_frame) != 0) {
                        queue_put(&s->empty_q, rgb_buffer);
                        break;
                    }
                    q_frame->rgb_data = rgb_buffer;
                    q_frame->pts = (dec_frame->pts == AV_NOPTS_VALUE) ? 0 : dec_frame->pts * av_q2d(time_base);
                    sws_scale(sws_ctx, (const uint8_t* const*)dec_frame->data, dec_frame->linesize, 0, v_codec_params->height, &q_frame->rgb_data, (const int[]){s->render_w * 3});

                    if (queue_put(&s->full_q, q_frame) != 0) {
                        // Main thread quit, clean up what we just allocated
                        queue_put(&s->empty_q, q_frame->rgb_data);
                        queue_put(&s->frame_pool_q, q_frame);
                        break;
                    }
                    av_frame_unref(dec_frame);
                }
            }
        } else if (pkt->stream_index == s->audio_stream_idx) {
            AVPacket* audio_pkt = av_packet_clone(pkt);
            if (queue_put(&s->audio_q, audio_pkt) != 0) {
                av_packet_free(&audio_pkt);
            }
        }
        av_packet_unref(pkt);
        if (s->quit || g_interrupted) break;
    }

    queue_finish(&s->full_q);
    queue_finish(&s->audio_q);

    av_packet_free(&pkt);
    av_frame_free(&dec_frame);
    sws_freeContext(sws_ctx);
    avcodec_free_context(&v_codec_ctx);
    return NULL;
}

// NEW: Forward declaration of get_master_clock for the audio thread
static inline double get_master_clock(PlayerState *s);

/**
 * The audio thread's job is to:
 * 1. Get an audio packet from the `audio_q`.
 * 2. Decode it, resample it to the format PulseAudio expects (S16LE).
 * 3. Synchronize against the master clock, waiting if audio is ahead of schedule.
 * 4. Write the samples to the PulseAudio stream.
 */
void* audio_thread_func(void* arg) {
    PlayerState* s = (PlayerState*)arg;
    AVStream* a_stream = s->format_ctx->streams[s->audio_stream_idx];
    AVCodecParameters* a_codec_params = a_stream->codecpar;
    const AVCodec* a_codec = avcodec_find_decoder(a_codec_params->codec_id);
    AVCodecContext* a_codec_ctx = avcodec_alloc_context3(a_codec);
    avcodec_parameters_to_context(a_codec_ctx, a_codec_params);
    if (avcodec_open2(a_codec_ctx, a_codec, NULL) < 0) { s->quit = true; return NULL; }

    pa_sample_spec pa_ss = { .format = PA_SAMPLE_S16LE, .channels = a_codec_ctx->ch_layout.nb_channels, .rate = a_codec_ctx->sample_rate };
    int pa_error;
    pa_simple* pa_s = pa_simple_new(NULL, "voxel", PA_STREAM_PLAYBACK, NULL, "playback", &pa_ss, NULL, NULL, &pa_error);
    if (!pa_s) { fprintf(stderr, "ERROR: pa_simple_new() failed: %s\n", pa_strerror(pa_error)); s->quit = true; return NULL; }
    s->pa_s = pa_s; // Share the handle with the main thread

    struct SwrContext* swr_ctx = NULL;
    swr_alloc_set_opts2(&swr_ctx, &a_codec_ctx->ch_layout, AV_SAMPLE_FMT_S16, pa_ss.rate, &a_codec_ctx->ch_layout, a_codec_ctx->sample_fmt, a_codec_ctx->sample_rate, 0, NULL);
    swr_init(swr_ctx);

    AVFrame* frame = av_frame_alloc();
    AVPacket* pkt;
    AVRational time_base = a_stream->time_base;

    const int MAX_SAMPLES_PER_FRAME = 4096;
    int resampled_buffer_size = av_samples_get_buffer_size(NULL, pa_ss.channels, MAX_SAMPLES_PER_FRAME, AV_SAMPLE_FMT_S16, 1);
    uint8_t *resampled_buffer = av_malloc(resampled_buffer_size);
    if (!resampled_buffer) {
        fprintf(stderr, "ERROR: Failed to allocate audio resample buffer.\n");
        s->quit = true;
        swr_free(&swr_ctx);
        avcodec_free_context(&a_codec_ctx);
        av_frame_free(&frame);
        pa_simple_free(pa_s);
        return NULL;
    }

    while (!s->quit && queue_get(&s->audio_q, (void**)&pkt) == 0) {
        if (avcodec_send_packet(a_codec_ctx, pkt) == 0) {
            while (avcodec_receive_frame(a_codec_ctx, frame) == 0) {
                if (frame->nb_samples > MAX_SAMPLES_PER_FRAME) {
                    fprintf(stderr, "Warning: Audio frame has %d samples, exceeding pre-allocated max of %d. Skipping.\n", frame->nb_samples, MAX_SAMPLES_PER_FRAME);
                    continue;
                }

                // This logic slaves the audio playback to the master system clock.
                if (frame->pts != AV_NOPTS_VALUE) {
                    double audio_pts = frame->pts * av_q2d(time_base);
                    double master_clock = get_master_clock(s);

                    useconds_t latency_us = pa_simple_get_latency(s->pa_s, &pa_error);
                    double latency_s = (latency_us == (useconds_t)-1) ? 0.0 : (double)latency_us / 1e6;

                    double target_master_clock = audio_pts - latency_s;
                    double delay = target_master_clock - master_clock;

                    if (delay > 0.0) {
                        struct timespec sleep_ts = { .tv_sec = (time_t)delay, .tv_nsec = (long)((delay - (time_t)delay) * 1e9) };
                        nanosleep(&sleep_ts, NULL);
                    }
                    // If audio is late (delay < 0), we play it immediately to try and catch up.
                }

                int out_samples = swr_convert(swr_ctx, &resampled_buffer, frame->nb_samples, (const uint8_t **)frame->data, frame->nb_samples);
                int out_buffer_size = av_samples_get_buffer_size(NULL, pa_ss.channels, out_samples, AV_SAMPLE_FMT_S16, 1);
                pa_simple_write(pa_s, resampled_buffer, out_buffer_size, &pa_error);
                s->audio_clock += (double)frame->nb_samples / a_codec_ctx->sample_rate;
            }
        }
        av_packet_free(&pkt);
    }

    avcodec_send_packet(a_codec_ctx, NULL); // Flush codec
    while (avcodec_receive_frame(a_codec_ctx, frame) == 0) {
        if (frame->nb_samples > MAX_SAMPLES_PER_FRAME) continue;
        int out_samples = swr_convert(swr_ctx, &resampled_buffer, frame->nb_samples, (const uint8_t **)frame->data, frame->nb_samples);
        int out_buffer_size = av_samples_get_buffer_size(NULL, pa_ss.channels, out_samples, AV_SAMPLE_FMT_S16, 1);
        pa_simple_write(pa_s, resampled_buffer, out_buffer_size, &pa_error);
        s->audio_clock += (double)frame->nb_samples / a_codec_ctx->sample_rate;
    }

    av_free(resampled_buffer);
    pa_simple_free(pa_s);
    s->pa_s = NULL;
    swr_free(&swr_ctx);
    avcodec_free_context(&a_codec_ctx);
    av_frame_free(&frame);
    return NULL;
}

// =====================================================================================
// == Main Function and Helpers
// =====================================================================================

double get_time_diff(const struct timespec *start, const struct timespec *end) {
    return (end->tv_sec - start->tv_sec) + (end->tv_nsec - start->tv_nsec) / 1e9;
}

int compare_doubles(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

void print_usage(const char *prog_name) {
    fprintf(stderr, "Usage: %s [options] <video_path>\n", prog_name);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  -w WIDTHxHEIGHT  Set a specific render PIXEL size\n");
    fprintf(stderr, "                     By default, the pixel size is set based on the terminal size\n");
    fprintf(stderr, "                     and the high-resolution mode used.\n");
    fprintf(stderr, "  -m MODE          Enable high-resolution mode. MODE can be:\n");
    fprintf(stderr, "                     half:     use half blocks for 2x vertical resolution\n");
    fprintf(stderr, "                     quadrant: use quadrant blocks for 2x vertical/horizontal resolution\n");
    fprintf(stderr, "  -t TOLERANCE     Perceptual color tolerance\n");
    fprintf(stderr, "                     Higher number means lower color quality and lower bandwidth.\n");
    fprintf(stderr, "  -a               Adaptive quality mode, overrides -t\n");
    fprintf(stderr, "  -u               Render frames as fast as possible, disabling audio\n");
    fprintf(stderr, "  -s               Show playback statistics upon completion\n");
    fprintf(stderr, "  -h               Show this help message\n");
    fprintf(stderr, "  -v               Show software version\n");
}

static inline double get_master_clock(PlayerState *s) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double current_time = ts.tv_sec + ts.tv_nsec / 1e9;

    if (s->playback_start_time < 0) {
        s->playback_start_time = current_time;
        return 0.0;
    }

    return current_time - s->playback_start_time;
}

static void free_av_packet(void* item) {
    AVPacket* pkt = (AVPacket*)item;
    av_packet_free(&pkt);
}

int main(int argc, char *argv[]) {
    char *video_path = NULL;
    int custom_w = 0, custom_h = 0;
    bool unlimited_mode = false;
    bool show_stats = false;
    RenderMode render_mode = MODE_FULL;
    int render_tolerance_sq = 0;
    bool adaptive_quality_mode = false;

    // --- Argument Parsing ---
    int opt;
    while ((opt = getopt(argc, argv, "w:m:t:aushv")) != -1) {
        switch (opt) {
            case 'w':
                if (sscanf(optarg, "%dx%d", &custom_w, &custom_h) != 2 || custom_w <= 0 || custom_h <= 0) {
                    fprintf(stderr, "ERROR: Invalid size format for -w. Use WIDTHxHEIGHT.\n");
                    return 1;
                }
                break;
            case 'm':
                if (strcmp(optarg, "half") == 0) {
                    render_mode = MODE_HALF;
                } else if (strcmp(optarg, "quadrant") == 0) {
                    render_mode = MODE_QUADRANT;
                } else {
                    fprintf(stderr, "ERROR: Unknown high-res mode '%s'. Use 'half' or 'quadrant'.\n", optarg);
                    return 1;
                }
                break;
            case 't':
                render_tolerance_sq = atoi(optarg);
                break;
            case 'u':
                unlimited_mode = true;
                break;
            case 's':
                show_stats = true;
                break;
            case 'a':
                adaptive_quality_mode = true;
                break;
            case 'v':
                fprintf(stderr, "voxel version 1.0.1\n");
                return 1;
            case 'h':
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (optind >= argc) {
        print_usage(argv[0]);
        return 1;
    }
    video_path = argv[optind];

    if (optind < argc - 1) {
        fprintf(stderr, "ERROR: Multiple video paths specified. Only one is allowed.\n");
        return 1;
    }

    av_log_set_level(AV_LOG_ERROR);
    signal(SIGINT, sigint_handler);

    // --- Player State and FFmpeg Initialization ---
    PlayerState player_state = {0};
    player_state.video_stream_idx = -1;
    player_state.audio_stream_idx = -1;
    player_state.playback_start_time = -1.0;
    queue_init(&player_state.full_q);
    queue_init(&player_state.empty_q);
    queue_init(&player_state.frame_pool_q);
    queue_init(&player_state.audio_q);

    if (avformat_open_input(&player_state.format_ctx, video_path, NULL, NULL) != 0) { fprintf(stderr, "ERROR: Could not open video file '%s'\n", video_path); return 1; }
    if (avformat_find_stream_info(player_state.format_ctx, NULL) < 0) { fprintf(stderr, "ERROR: Could not find stream info for '%s'\n", video_path); return 1; }

    for (unsigned int i = 0; i < player_state.format_ctx->nb_streams; i++) {
        if (player_state.format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO && player_state.video_stream_idx < 0) player_state.video_stream_idx = i;
        if (player_state.format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO && player_state.audio_stream_idx < 0) player_state.audio_stream_idx = i;
    }
    if (player_state.video_stream_idx == -1) { fprintf(stderr, "ERROR: No video stream found in file.\n"); return 1; }
    if (unlimited_mode) player_state.audio_stream_idx = -1; // Disable audio

    // --- Calculate Render Dimensions ---
    AVCodecParameters* v_params = player_state.format_ctx->streams[player_state.video_stream_idx]->codecpar;
    int render_w = custom_w, render_h = custom_h;
    if (render_w == 0 || render_h == 0) {
        double video_aspect = (double)v_params->width / v_params->height;
        struct winsize ws; ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws);
        int term_w = ws.ws_col, term_h = ws.ws_row > 1 ? ws.ws_row - 1 : 1;
        double term_aspect_char = (double)term_w / term_h;
        double effective_aspect = video_aspect * 2.0; // Assume font is ~2x taller than wide
        int char_w, char_h;
        if (term_aspect_char > effective_aspect) { char_h = term_h; char_w = (int)round(term_h * effective_aspect); }
        else { char_w = term_w; char_h = (int)round(term_w / effective_aspect); }
        switch (render_mode) {
            case MODE_FULL: render_w = char_w; render_h = char_h; break;
            case MODE_HALF: render_w = char_w; render_h = char_h * 2; break;
            case MODE_QUADRANT: render_w = char_w * 2; render_h = char_h * 2; break;
        }
    }
    if (render_mode == MODE_HALF && render_h % 2 != 0) render_h--;
    if (render_mode == MODE_QUADRANT) { if (render_w % 2 != 0) render_w--; if (render_h % 2 != 0) render_h--; }
    if (render_w < 1) render_w = 1; if (render_h < 1) render_h = 1;
    player_state.render_w = render_w;
    player_state.render_h = render_h;

    double video_frame_duration = 0.0;
    AVStream* v_stream = player_state.format_ctx->streams[player_state.video_stream_idx];
    if (!unlimited_mode) {
        double frame_rate = av_q2d(v_stream->avg_frame_rate);
        if (frame_rate > 0) {
            video_frame_duration = 1.0 / frame_rate;
        } else { // Fallback, likely not work
            video_frame_duration = 1.0 / 25.0;
        }
    }

    // --- Create Buffer Pool and Threading Resources ---
    size_t rgb_buffer_size = render_w * render_h * 3;
    uint8_t** rgb_buffer_pool = malloc(QUEUE_CAPACITY * sizeof(uint8_t*));
    for (int i = 0; i < QUEUE_CAPACITY; ++i) {
        rgb_buffer_pool[i] = malloc(rgb_buffer_size);
        queue_put(&player_state.empty_q, rgb_buffer_pool[i]);
    }
    QueuedFrame** frame_pool = malloc(QUEUE_CAPACITY * sizeof(QueuedFrame*));
    for (int i = 0; i < QUEUE_CAPACITY; ++i) {
        frame_pool[i] = malloc(sizeof(QueuedFrame));
        queue_put(&player_state.frame_pool_q, frame_pool[i]);
    }
    uint8_t *prev_rgb_buffer = malloc(rgb_buffer_size);
    memset(prev_rgb_buffer, 0, rgb_buffer_size);

    double* render_times = show_stats ? malloc(MAX_RENDER_TIMES * sizeof(double)) : NULL;
    long frame_count = 0;
    double total_render_time = 0.0;

    // --- Start Threads ---
    pthread_t decode_tid, audio_tid;
    pthread_create(&decode_tid, NULL, decode_thread_func, &player_state);
    if (player_state.audio_stream_idx != -1) {
        pthread_create(&audio_tid, NULL, audio_thread_func, &player_state);
    }

    // --- Main Render Loop (Consumer) ---
    TerminalContext term = {0};
    buffer_append(&term, "\x1b[?25l\x1b[2J"); // Hide cursor, clear screen
    buffer_flush(&term);

    struct timespec total_start_ts, total_end_ts;
    struct rusage start_usage, end_usage;
    clock_gettime(CLOCK_MONOTONIC, &total_start_ts);
    getrusage(RUSAGE_SELF, &start_usage);

    QueuedFrame* q_frame;
    while (!g_interrupted && queue_get(&player_state.full_q, (void**)&q_frame) == 0) {
        if (q_frame->pts > 0 && !unlimited_mode) {
            double master_clock = get_master_clock(&player_state);
            double delay = q_frame->pts - master_clock;

            if (delay < -video_frame_duration) {
                queue_put(&player_state.empty_q, q_frame->rgb_data);
                queue_put(&player_state.frame_pool_q, q_frame);
                continue;
            }

            if (delay > 0.0) {
                struct timespec sleep_ts = {0};
                sleep_ts.tv_sec = (time_t)delay;
                sleep_ts.tv_nsec = (long)((delay - sleep_ts.tv_sec) * 1e9);
                nanosleep(&sleep_ts, NULL);
            }
        }

        struct timespec frame_start_ts, frame_end_ts;
        bool should_time_frame = show_stats || (adaptive_quality_mode && !unlimited_mode);

        if (should_time_frame) clock_gettime(CLOCK_MONOTONIC, &frame_start_ts);

        term.cursor_row = 0; term.cursor_col = 0;
        switch (render_mode) {
            case MODE_FULL: render_frame_full_block(&term, render_w, render_h, q_frame->rgb_data, prev_rgb_buffer, frame_count, render_tolerance_sq); break;
            case MODE_HALF: render_frame_half_block(&term, render_w, render_h, q_frame->rgb_data, prev_rgb_buffer, frame_count, render_tolerance_sq); break;
            case MODE_QUADRANT: render_frame_quadrant(&term, render_w, render_h, q_frame->rgb_data, prev_rgb_buffer, frame_count, render_tolerance_sq); break;
        }
        memcpy(prev_rgb_buffer, q_frame->rgb_data, rgb_buffer_size);
        buffer_flush(&term);
        frame_count++;

        if (should_time_frame) {
            clock_gettime(CLOCK_MONOTONIC, &frame_end_ts);
            double render_duration = get_time_diff(&frame_start_ts, &frame_end_ts);

            if (adaptive_quality_mode && !unlimited_mode && video_frame_duration > 0) {
                if (render_duration > video_frame_duration * 1.10) {
                    render_tolerance_sq += 10;
                } else if (render_duration < video_frame_duration * 0.90) {
                    render_tolerance_sq -= 10;
                    if (render_tolerance_sq < 0) render_tolerance_sq = 0;
                }
            }

            if (show_stats) {
                total_render_time += render_duration;
                if (frame_count <= MAX_RENDER_TIMES) {
                    render_times[frame_count - 1] = render_duration;
                }
            }
        }

        // Return the RGB buffer and the frame container to their respective pools
        queue_put(&player_state.empty_q, q_frame->rgb_data);
        queue_put(&player_state.frame_pool_q, q_frame);
    }
    player_state.quit = true; // Signal other threads to exit

    clock_gettime(CLOCK_MONOTONIC, &total_end_ts);
    getrusage(RUSAGE_SELF, &end_usage);

    // --- Cleanup and Shutdown ---
    queue_finish(&player_state.full_q);
    queue_finish(&player_state.empty_q);
    if (player_state.audio_stream_idx != -1) {
        queue_finish(&player_state.audio_q);
    }

    // Now join the threads
    pthread_join(decode_tid, NULL);
    if (player_state.audio_stream_idx != -1) {
        pthread_join(audio_tid, NULL);
    }

    buffer_append(&term, "\x1b[?25h\x1b[0m\n"); // Show cursor, reset attributes
    buffer_flush(&term);

    // --- Print Statistics ---
    if (show_stats) {
        double total_duration = get_time_diff(&total_start_ts, &total_end_ts);
        printf("--- Playback Statistics ---\n");
        const char* mode_str = "Full-Block (1x1)";
        if (render_mode == MODE_HALF) mode_str = "Half-Block (1x2)";
        else if (render_mode == MODE_QUADRANT) mode_str = "Quadrant (2x2)";
        printf("Render Mode:          %s\n", mode_str);

        int term_w = render_w, term_h = render_h;
        if (render_mode == MODE_HALF) term_h /= 2;
        else if (render_mode == MODE_QUADRANT) { term_w /= 2; term_h /= 2; }
        printf("Pixel/Char Size:      %dpx W x %dpx H -> %dch W x %dch H\n", render_w, render_h, term_w, term_h);
        if (adaptive_quality_mode)
            printf("Render Tolerance:     Adaptive\n");
        else
            printf("Render Tolerance:     %d\n", render_tolerance_sq);
        printf("Playback Speed:       %s\n", unlimited_mode ? "Unlimited" : "Normal");

        if (frame_count > 0 && total_duration > 0) {
            if (!unlimited_mode) {
                double avg_fps = frame_count / total_duration;
                printf("Average FPS:          %.2f FPS\n", avg_fps);
            } else {
                double avg_render_ms = (total_render_time / frame_count) * 1000.0;
                printf("Average render time:  %.2f ms (%.2f FPS equiv.)\n", avg_render_ms, 1000.0 / avg_render_ms);

                if (frame_count > 10) {
                    size_t num_times = frame_count < MAX_RENDER_TIMES ? frame_count : MAX_RENDER_TIMES;
                    qsort(render_times, num_times, sizeof(double), compare_doubles);
                    double low_percentile_time_s = render_times[(int)(num_times * 0.99)];
                    printf("1%% low render time:   %.2f ms (%.2f FPS equiv.)\n", low_percentile_time_s * 1000.0, 1.0 / low_percentile_time_s);
                }
            }

            double term_bw_mbps = (term.bytes_written / total_duration) * 8.0 / (1024 * 1024);
            printf("Terminal Bandwidth:   %.2f Mbps\n", term_bw_mbps);

            long user_sec = end_usage.ru_utime.tv_sec - start_usage.ru_utime.tv_sec;
            long user_usec = end_usage.ru_utime.tv_usec - start_usage.ru_utime.tv_usec;
            long sys_sec = end_usage.ru_stime.tv_sec - start_usage.ru_stime.tv_sec;
            long sys_usec = end_usage.ru_stime.tv_usec - start_usage.ru_stime.tv_usec;
            double cpu_time = (user_sec + sys_sec) + (user_usec + sys_usec) / 1e6;
            printf("Total Process CPU:    %.2f%% (%.2fs CPU time on 1 core)\n", (cpu_time / total_duration) * 100.0, cpu_time);
        }
    }

    // --- Final Resource Freeing ---
    free(prev_rgb_buffer);
    free(render_times);
    for (int i = 0; i < QUEUE_CAPACITY; ++i) {
        free(rgb_buffer_pool[i]);
        free(frame_pool[i]);
    }
    free(rgb_buffer_pool);
    free(frame_pool);

    queue_destroy(&player_state.full_q, NULL);
    queue_destroy(&player_state.audio_q, free_av_packet); // Frees remaining AVPacket clones
    queue_destroy(&player_state.empty_q, NULL); // Just destroys mutex/cond, items are from the pool
    queue_destroy(&player_state.frame_pool_q, NULL);
    avformat_close_input(&player_state.format_ctx);

    return 0;
}
