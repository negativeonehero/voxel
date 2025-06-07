# Voxel
Voxel is a terminal video player written in C. It's simple, fast and widely compatible.

## How fast is it?
I compared Voxel vs. two other well-known solutions: [TheRealOrange/terminalvideoplayer](https://github.com/TheRealOrange/terminalvideoplayer) and `mpv`'s tct with a 30-second Pringles commercial on Windows Terminal with a Ryzen 4800H CPU:

|                |Setup                                     |Resolution               |FPS average/1% lows                              |
|----------------|------------------------------------------|-------------------------|-------------------------------------------------|
|mpv             |--vo=tct                                  |377x106 (1x2 half-block) |- (261/720 frames dropped)                       |
|tvp             |color threshold 10, custom unlimited mode |377x106 (4x4 quarter)    |38.4 FPS/-                                       |
|tvp             |color threshold 40, custom unlimited mode |377x106 (4x4 quarter)    |58.0 FPS/- (noticeable quality dropoff)          |
|Voxel           |-t 10 -h quadrant -u                      |377x106 (2x2 quadrant)   |47.9 FPS/33.2 FPS                                |
|Voxel           |-t 50 -h quadrant -u                      |377x106 (2x2 quadrant)   |**69.4** FPS/37.9 FPS (moderate quality dropoff) |

It's fast. In fact, it can be as fast as you want*.

## Why is it so fast?
Because tvp confused the hell out of me.

Just kidding, there are a ton of optimizations focusing solely on the most important metric in terminal rendering: bandwidth. To name a few:
- Delta framing: also used in tvp, this optimization greatly reduces bandwidth by not redrawing pixels unchanged or changed very little between the frames. The threshold is set by the -t flag.
- Run-length encoding: also used in tvp, this optimization draws several contiguous pixels in one escape sequence.
- Color swapping: in high-resolution modes, background and foreground colors can be swapped. This reduces the number of masks, or can bring close colors together which is beneficial for RLE.
- Smart cursor movement: attempts to use new lines (`\n`) and relative cursor movements (`\x1b[1C`) over absolute cursor movements (`\x1b[2;2H`).

and more...

## How to compile and run
You'll need ffmpeg and PulseAudio (to play audio).
To compile Voxel, run this command:
```
gcc voxel.c -o voxel -lavcodec -lavformat -lavutil -lswscale -lswresample -lpulse -lpulse-simple -lm -O3
```
Run the `voxel` command without any parameters to learn about the usage.

