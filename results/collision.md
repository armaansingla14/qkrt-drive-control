Random circles and oriented boxes (~1 obstacle per 25 m^2), 20000 oriented-box rover footprints per size.

| Obstacles | Method | Build [ms] | Query all hits [ns] | Any-hit [ns] | Speed-up (all hits) | Hits match brute force |
|---|---|---|---|---|---|---|
| 100 | brute_force | 0.04 | 231 | 214 | 1.0x | yes |
| 100 | bvh | 0.08 | 157 | 153 | 1.5x | yes |
| 100 | aabb_tree | 0.06 | 181 | 172 | 1.3x | yes |
| 1000 | brute_force | 0.10 | 2368 | 2139 | 1.0x | yes |
| 1000 | bvh | 0.71 | 244 | 230 | 9.7x | yes |
| 1000 | aabb_tree | 0.53 | 405 | 370 | 5.9x | yes |
| 10000 | brute_force | 1.07 | 43143 | 39787 | 1.0x | yes |
| 10000 | bvh | 8.23 | 430 | 393 | 100.4x | yes |
| 10000 | aabb_tree | 7.79 | 1059 | 931 | 40.7x | yes |
| 100000 | brute_force | 10.26 | 475156 | 423217 | 1.0x | yes |
| 100000 | bvh | 93.84 | 944 | 629 | 503.2x | yes |
| 100000 | aabb_tree | 109.12 | 2592 | 2411 | 183.3x | yes |

Dynamic scene: 10000 obstacles, 1000 move per frame (~5 cm), 1000 queries per frame, 100 frames.

| Strategy | ms per frame | Hits match |
|---|---|---|
| brute_force | 45.548 | yes |
| bvh_rebuild_each_frame | 6.346 | yes |
| aabb_tree_incremental | 1.358 | yes |
