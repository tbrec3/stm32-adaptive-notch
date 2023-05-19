# Adaptive Notch Filter

This repository contains the implementation of an adaptive notch filter using the recursive gradient descent algorithm.
The CubeIDE project provides an example to filter single sinusoidal noise from an input signal, given the isolated sinusoid at the reference input.

Using the DMA and double buffers, the input samples are processed block-wise to offload the CPU and allow for additional tasks to be run.