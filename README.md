# NNSearch
Nearest neighbors search using a kd-tree

## Timings
I tested timing on Archlinux (linux kernel 5.2.4) using gcc 9.1.0 with optimizations. My CPU is and Intel Core i7-6700K.
The test runs 10000 random queries against the loaded mesh.

| Shape | Nb. of Points | Tree Construction Time | Tree Memory | Average Time |
|:-|:-:|:-:|:-:|:-:|
| Stanford Bunny | 14904 (0.341125mb) | 3.3495 ms| 1.24992mb |  9.92943 µs |
| Stanford Dragon | 2613918 (59.8278mb) | 744.452 ms | 320mb | 680.913 µs |
| Happy Buddha | 3262422 (74.6707mb) | 954.633 ms | 320mb | 549.162 µs |
| Random Points | 15000000 (343.323mb) | 10707.3 ms | 1280mb | 8883.77 µs |
