# NNSearch
Nearest neighbors search using a kd-tree

## Timings
I tested timing on Archlinux (linux kernel 5.2.4) using gcc 9.1.0 with optimizations. My CPU is and Intel Core i7-6700K.
The test runs 10000 random queries against the loaded mesh.

| Shape | Nb. of Points | Tree Construction Time | Tree Memory | Total Time | Average Time |
|:-|:-:|:-:|:-:|:-:|:-:|
| Stanford Bunny | 14904 (0.341125mb) | 3.3495 ms| 1.24992mb |  0.117759 s | 0.0117759 ms |
| Stanford Dragon | 2613918 (59.8278mb) | 748.572 ms | 320mb | 7.36239 s | 0.736239 ms |
| Happy Buddha | 3262422 (74.6707mb) | 954.633 ms | 320mb | 6.11195 s | 0.611195 ms |
