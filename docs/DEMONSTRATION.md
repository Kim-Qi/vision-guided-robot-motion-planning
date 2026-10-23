# Demonstration media

The repository includes an actual full-run [red-target GIF](../assets/pick_place_red_preview.gif), accelerated 4×, plus final frames for red/blue/green. It also retains the existing [pre-grasp MP4](../assets/demo.mp4). These are actual renders, not generated mockups.

Full recordings have been finalized, ffprobe-validated, fully decoded, and manually reviewed by the user:

| Asset prepared for release | Accepted runtime run | Format | Duration |
|---|---|---|---:|
| `red_cube_pick_place.mp4` | multi_red_016 | H.264 1280×720 / 10 fps | 75.7 s |
| `blue_cube_pick_place.mp4` | multi_blue_015 | H.264 1280×720 / 10 fps | 87.0 s |
| `green_cube_pick_place.mp4` | multi_green_017 | H.264 1280×720 / 10 fps | 80.4 s |
| `pick_place_evidence.tar.gz` | Three successes and two failures | Original numeric telemetry + compact records | — |

The original full videos show initial objects, selected target, approach, closing, physical lift, transfer, placement, release and retreat. Video sampling can shorten playback relative to wall time; exact task/orchestration times are in experiment records.

**Publication status:** the full assets are prepared outside Git history but not uploaded. GitHub authentication is unavailable in the working environment; no release/download URL is represented as active. Repository/PR publishing is also pending authentication. After the final publication commit is reviewed/merged and tagged `v1.0.0-pick-place`, upload these assets to that release and replace this status with actual URLs. Never create a stable tag pointing to an unconfirmed commit.
