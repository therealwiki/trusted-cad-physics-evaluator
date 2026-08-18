#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FFMPEG=/opt/homebrew/bin/ffmpeg
WORK="$ROOT/render/timeline"
OUT="$ROOT/outputs"
mkdir -p "$WORK" "$OUT"

still_segment() {
  local card="$1" duration="$2" output="$3"
  "$FFMPEG" -y -loop 1 -framerate 30 -i "$ROOT/render/cards/$card.png" -t "$duration" \
    -vf "scale=1920:1080,fade=t=in:st=0:d=.35,fade=t=out:st=$(awk -v d="$duration" 'BEGIN{print d-.35}'):d=.35,format=yuv420p" \
    -r 30 -c:v libx264 -crf 15 -preset medium -an "$WORK/$output.mp4"
}

clip_segment() {
  local clip="$1" duration="$2" output="$3"
  "$FFMPEG" -y -i "$ROOT/render/$clip.mp4" -t "$duration" \
    -vf "scale=1920:1080,tpad=stop_mode=clone:stop_duration=$duration,fade=t=in:st=0:d=.25,fade=t=out:st=$(awk -v d="$duration" 'BEGIN{print d-.35}'):d=.35,format=yuv420p" \
    -r 30 -c:v libx264 -crf 15 -preset medium -an "$WORK/$output.mp4"
}

still_segment 01_empty 8 01
still_segment 02_contract 10 02
still_segment 03_pipeline 7 03
still_segment 04_material 6 04
still_segment 05_attempts_a 10 05
clip_segment candidate_006_actual 6 06
still_segment 06_attempts_b 8 07
clip_segment candidate_011_actual 6 08
still_segment 07_attempts_c 8 09
clip_segment candidate_013_actual 6 10
clip_segment candidate_014_final_actual 18 11
still_segment 10_evidence 8 12
still_segment 11_end 4 13

: > "$WORK/concat.txt"
for index in $(seq -w 1 13); do printf "file '%s.mp4'\n" "$index" >> "$WORK/concat.txt"; done
"$FFMPEG" -y -f concat -safe 0 -i "$WORK/concat.txt" -c copy "$WORK/picture.mp4"
"$FFMPEG" -y -f lavfi -i "sine=frequency=55:sample_rate=48000:duration=105" \
  -f lavfi -i "anoisesrc=color=pink:sample_rate=48000:duration=105" \
  -filter_complex "[0:a]volume=.035[a0];[1:a]lowpass=f=700,volume=.012[a1];[a0][a1]amix=inputs=2,afade=t=in:st=0:d=2,afade=t=out:st=102:d=3[a]" \
  -map "[a]" -c:a aac -b:a 192k "$WORK/soundtrack.m4a"
"$FFMPEG" -y -i "$WORK/picture.mp4" -i "$WORK/soundtrack.m4a" -map 0:v -map 1:a \
  -c:v copy -c:a aac -shortest -movflags +faststart "$OUT/final_gearbox_evaluator_1080p.mp4"
"$FFMPEG" -y -i "$OUT/final_gearbox_evaluator_1080p.mp4" \
  -vf scale=3840:2160:flags=lanczos -c:v libx264 -preset slow -crf 16 -c:a copy -movflags +faststart \
  "$OUT/final_gearbox_evaluator_4k.mp4"
cp "$ROOT/render/cards/11_end.png" "$OUT/poster_frame.png"
