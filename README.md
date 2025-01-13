# Ray Tracing in Rust
#### Video Demo: https://youtu.be/NpVQilrxkjg
#### Description: A simple raytracer implemented in Rust based on "[Raytracing in One Weekend](https://raytracing.github.io)"

![image](https://github.com/user-attachments/assets/8507e9db-37ba-4aa1-9f46-5348af4aa03c)

## Background

This raytracer was written for the final project of [Harvard CS50x](https://cs50.harvard.edu/x/). My initial exposure to programming was making little games in ProcessingJS. I have always liked the idea of 3D graphics but found it intimidating. This project was a chance for me to delve into that field a little deeper and learn about how 3D works. 

I chose to implement this project in the Rust programming language simply because I have wanted to learn Rust for a long time. Before taking CS50x, however, I struggled to understand the specifics of Rust—especially its strong and static type system, borrowing system, and use of pointers and references. Thanks to CS50x, I understood those concepts much better and was prepared to learn and use Rust this time around. And besides, Rust is fast and fun to use once I understood how it works!

## First Attempt

Initially, I wrote a raytracer that closely followed Peter Shirley's first book of his ray tracing trilogy. I translated the C++ code into Rust and slowly assimilated the concepts taught in the book. This first attempt helped me practice basic Rust, especially the use of traits and trait objects to emulate the object-oriented programming idea of polymorphism.

There were a few things that I wanted to improve on, though. For one, Shirley's first book did not teach how to render triangles or models constructed from meshes of triangles. I also thought that I could speed up the rendering time by implementing a Bounding Volume Hierarchy (BVH). Finally, at that time I disliked the idea that I was using trait objects in order to store different types of shapes in the same vector. I thought that that slowed my program down, though now I know that using trait objects can actually improve code structure and readability and that there were other areas of the program that could be optimized instead.

## Project

I also switched computers at this time, so instead of moving my original project to my new computer I decided to re-write the project from ground-up. This part was not very difficult because I already understood fairly well how the raytracer worked and I also had the help of (a free trial of) GitHub Copilot. I also used git to keep track of my program's progress, so the commit history of this project shows the progression my program made. There were a few key differences between this project and the initial one:
- Instead of using a `Hittable` trait and implementing that trait for each shape type, I implemented a `Hittable` object that contained a shape and material type.
- Instead of writing my own vector library, I utilized the `nalgebra` crate for vector math. Hopefully the library is more efficient than my own implementation.
- Instead of splitting the program into multiple files, I left it all in one file (that was a bad idea).

I also added some features that were not specifically covered in Shirley's book:
- Parallelism (which was fairly easy with the `rayon` crate)
- Triangle models (.stl file input and parsing handled by `stl_io` crate)

The biggest upgrade in the second project was the implementation of a Bounding Volume Hierarchy (BVH), however. Originally, every single ray was checking for intersection with every shape in the scene. A low-resolution image (400-pixel width, 10 rays per pixel) has 900,000 rays to check collisions with every shape in the scene. And even low-poly triangle meshes still have many triangles in them. This really adds up when it comes to rendering speed.

A BVH sorts all the objects of the scene into a binary search tree. This changes runtime from linear to logarithmic and provides an incredible speed boost when it comes to ray-intersection checking. For example, before implementing BVH, I rendered a super high quality scene; it took about 21 hours. After BVH, a similar scene with similar image quality took *less than one hour*. Tremendous improvement! And with BVH, the raytracer is able to handle the Stanford dragon model (80K triangles; courtesy of [Sebastian Lague](https://github.com/SebLague/Ray-Tracing/tree/main/Assets/Graphics); the knight model also comes from him) in a reasonable amount of time.

## Concluding Thoughts

I really enjoyed writing this raytracer. There are still some features that could be implemented, such as:
- Transformable triangle models (currently they can only be positioned at the origin)
- Transformable box shapes
- Cornell box setup
- Lighting
- Dielectrics
- Benchmarking (speed tests)

But I'll have to save them for some other time.

Thanks for reading this!

![image2](https://github.com/user-attachments/assets/be7435eb-0d58-459e-9032-e8c690753f85)
