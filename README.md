#EDLine

Use AVX2/SSE2 accelerated line segment extraction algorithm, the original algorithm is EDLines.

The biggest improvement of this algorithm is to use the instruction set to speed up the calculation of Sobel,
Calculate the three gradients of dx, dy and grad at the same time in Sobel.hpp, avoiding the repeated calculation of intermediate quantities and memory allocation when using OpenCV,
Compared to the original algorithm, the time is reduced from 8ms to 2ms.

By modifying the conditional compilation options of Sobel.hpp, you can experiment with time-consuming under different conditions.
Extract line segments from a 1280*720 grayscale image. The CPU used in the experiment is Intel® Core™ i7-4710MQ CPU @ 2.50GHz × 8:
1. When using the AVX2 instruction set, it takes about 1.8ms;
2. When using the SSE2 instruction set, it takes about 1.96ms;
3. When using the implementation of OpenCV, it takes about 5.56ms;
4. When using handwritten implementation, it takes about 4.87ms;
5. The original algorithm takes about 8ms.


Instructions:

1. Display the line segment extraction effect:
  ./show ../imagge/00.png

2. View the time-consuming extraction of line segments (1000 cycles are used in the program, alternately extracting different images)
  ./time_cost ../image/l01.png ../image/r01.png

使用AVX2/SSE2加速的线段提取算法，原始算法为EDLines。

本算法最大的改进在于使用指令集加速了Sobel的计算，在Sobel.hpp中
同时计算dx,dy和grad三种梯度，避免了使用OpenCV时重复计算中间量和内存分配，
相比原始算法，耗时从 8ms 减少到 2ms。

通过修改Sobel.hpp的条件编译选项，可以实验不同条件下的耗时。
在1280*720的灰度图像提取线段，实验用的CPU为Intel® Core™ i7-4710MQ CPU @ 2.50GHz × 8 ：
1.使用AVX2指令集时，耗时为 1.8ms 左右；
2.使用SSE2指令集时，耗时为 1.96ms 左右；
3.使用OpenCV的实现时，耗时为 5.56ms 左右；
4.使用手写的实现时，耗时为 4.87ms 左右；
5.原始算法耗时为 8ms 左右。


使用方法：

1.显示线段提取效果
  ./show ../imagge/00.png

2.查看提取线段的耗时（程序中使用了1000次循环，交替提取不同图像）
  ./time_cost ../image/l01.png ../image/r01.png
