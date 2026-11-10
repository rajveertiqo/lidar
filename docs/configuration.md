# Configuration

Everything lives in `config.py`, grouped by subsystem. The values worth knowing:

| Setting                             | Meaning                                                                                              |
| ----------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `GUIDANCE_DANGER_MM` = 800          | Imminent — stop                                                                                      |
| `GUIDANCE_WARNING_MM` = 1500        | Close — caution                                                                                      |
| `WALK_BLOCK_FT` = 4.0               | Front nearer than this is blocked (6.5 until 2026-10-01)                                             |
| `WALK_MIN_TURN_FT` = 2.0            | Room a turn target must have, else stop                                                              |
| `WALK_GLASS_GAP_FT` = 2.0           | Sensors disagreeing by this ⇒ glass (the nearer wins either way, since 2026-10-30)                   |
| `WALK_FUSION_GLASS_OVERRIDE` = True | False drops the "sensors disagree" explanation; the nearer still wins (it averaged until 2026-10-30) |
| `WALK_TURN_MARGIN_FT` = 1.5         | A turn must beat the forward clearance by this, else keep going                                      |
| `LIDAR_NO_RETURN_FT` = 40           | Silent LiDAR counts as this much clear                                                               |
| `RADAR_NO_RETURN_FT` = 13.1         | Silent radar counts as this much clear (= `RADAR_END_MM`, 4 m since 2026-10-01; was 9.8)                                                               |
| `DRIFT_ALERT_TTC_S` = 5.0           | Seconds-to-contact that starts a drift nudge                                                         |
| `DRIFT_URGENT_TTC_S` = 2.5          | Seconds-to-contact that makes it firm                                                                |
| `DRIFT_REPEAT_S` = 3.0              | Same drift cue not repeated inside this window                                                       |
| `DRIFT_ANALYZE_HZ` = 8.0            | Rate cap — it shares the LiDAR thread with the stop decision                                         |
| `SEG_TARGET_FPS` = 10.0             | Segmentation rate cap on the shared NPU                                                              |
| `LIDAR_STALE_S` = 1.0               | Same rule for a scan read outside the thread that produced it                                        |
| `LIDAR_MIRROR` = False              | **Verified — do not flip.** True sends "turn RIGHT" to the left                                      |

Guidance thresholds are imported by _both_ the navigation and fusion layers, so
the two cannot drift apart.
