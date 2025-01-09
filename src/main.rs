use core::f32;
use std::cmp;
use std::fs::{File, OpenOptions};
use std::io::{Write, BufWriter};
use std::sync;


extern crate nalgebra as na;
use na::Vector3;

extern crate indicatif;
use indicatif::{ProgressBar, MultiProgress};

extern crate rand;

extern crate rayon;
use rayon::prelude::*;

// Functions related to vectors
fn random_unit_vector() -> Vector3<f32> {
    loop {
        let vector = Vector3::<f32>::new(rand::random::<f32>() * 2.0 - 1.0, rand::random::<f32>() * 2.0 - 1.0, rand::random::<f32>() * 2.0 - 1.0);
        let lensq = vector.magnitude_squared();

        if f32::MIN_POSITIVE < lensq && lensq < 1.0 {
            return vector / lensq;
        }
    }
}
fn random_on_hemisphere(normal: &Vector3<f32>) -> Vector3<f32> {
    let vector = random_unit_vector();
    if vector.dot(normal) > 0.0 {
        vector
    } else {
        -vector
    }
}
fn near_zero(vector: &Vector3<f32>) -> bool {
    let s = 1e-8;
    vector.x.abs() < s && vector.y.abs() < s && vector.z.abs() < s
}
fn reflect(vector: &Vector3<f32>, normal: &Vector3<f32>) -> Vector3<f32> {
    vector - 2.0 * vector.dot(normal) * normal
}
fn random_in_unit_disk() -> Vector3<f32> {
    loop {
        let vector = Vector3::<f32>::new(rand::random::<f32>() * 2.0 - 1.0, rand::random::<f32>() * 2.0 - 1.0, 0.0);
        let lensq = vector.magnitude_squared();

        if f32::MIN_POSITIVE < lensq && lensq < 1.0 {
            return vector;
        }
    }
}

// Functions related to color
fn linear_to_gamma(value: f32) -> f32 {
    value.sqrt()
}
fn write_color(image_buffer: &mut BufWriter<Vec<u8>>, pixel_color: &Vector3<f32>) -> std::io::Result<()> {
    let r = linear_to_gamma(pixel_color.x);
    let g = linear_to_gamma(pixel_color.y);
    let b = linear_to_gamma(pixel_color.z);

    let intensity = Interval { min: 0.0, max: 0.999};
    let rbyte = (256.0 * intensity.clamp(r)) as i32;
    let gbyte = (256.0 * intensity.clamp(g)) as i32;
    let bbyte = (256.0 * intensity.clamp(b)) as i32;

    image_buffer.write(format!("{rbyte} {gbyte} {bbyte}\n").as_bytes())?;

    Ok(())
}

// Functions related to models
fn import_stl_model(path: &str) -> Vec<Triangle>{
    let mut file = OpenOptions::new().read(true).open(path).unwrap();

    let stl = stl_io::create_stl_reader(&mut file).unwrap().map(|triangle| {
        let tri = triangle.expect("error");
        
        let vertices: Vec<Vector3<f32>> = tri.vertices.into_iter().map(|vertex| {
            Vector3::<f32>::new(vertex[0], vertex[1], vertex[2])
        }).collect();

        let normal = Vector3::<f32>::new(tri.normal[0], tri.normal[1], tri.normal[2]);

        Triangle {
            vertices: [vertices[0], vertices[1], vertices[2]],
            normal
        }
    }).collect();

    stl
}
fn add_model_to_world(world: &mut HittableList, stl_triangles: Vec<Triangle>, material: sync::Arc<Material>) {
    for tri in stl_triangles {
        world.add(Hittable {
            geometry: Geometry::Triangle(tri),
            material: material.clone()
        });
    }
}

// Function related to rays
struct Ray {
    origin: Vector3<f32>,
    direction: Vector3<f32>,
}
impl Ray {
    fn at(&self, t: f32) -> Vector3<f32> {
        self.origin + t * self.direction
    }
}

// Functions related to geometry
struct Triangle {
    vertices: [Vector3<f32>; 3],
    normal: Vector3<f32>
}
struct Sphere {
        pub center: Vector3<f32>,
        pub radius: f32,
    }

enum Geometry {
    Triangle(Triangle),
    Sphere(Sphere),
}
impl Geometry {
    fn hit(&self, ray: &Ray, ray_t: Interval, hit_record: &mut HitRecord) -> bool {
        match self {
            Geometry::Triangle(triangle) => {
                // This function uses the Moller-Trumbore algorithm to calculate the
                // intersection between a ray and a triangle.

                let v1 = triangle.vertices[0];
                let v2 = triangle.vertices[1];
                let v3 = triangle.vertices[2];

                let e1 = v2 - v1;
                let e2 = v3 - v1;

                // Normal of the triangle
                let p = ray.direction.cross(&e2);

                // Calculate the determinant (angle betweeen the ray direction and the triangle)
                let det = e1.dot(&p);

                // If the ray direction is parallel to the triangle, they don't intersect.
                if det == 0.0 {
                    return false;
                }

                // Compute the inverse determinant
                let inv_det = 1.0 / det;

                // Calculate the vector from the ray origin to a triangle vertex
                let t = ray.origin - v1;

                // Calculate the u parameter
                let u = t.dot(&p) * inv_det;

                // Calculate the cross product for the v parameter
                let q = t.cross(&e1);

                // Calculate the v parameter
                let v = ray.direction.dot(&q) * inv_det;

                // Calculate the t parameter
                let t = e2.dot(&q) * inv_det;

                // Make sure that the triangle is in front of the ray
                // and that the intersection is inside the triangle
                if t > 0.0 && u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0 && u + v <= 1.0 {

                    if !ray_t.surrounds(t) {
                        return false;
                    }

                    let outward_normal = triangle.normal;

                    hit_record.t = t;
                    hit_record.p = ray.at(t);
                    hit_record.set_face_normal(ray, outward_normal);
                    
                    return true;
                } else {
                    return false;
                }
            },
            Geometry::Sphere(sphere) => {
                let oc = sphere.center - ray.origin;
                let a = ray.direction.magnitude_squared();
                let h = ray.direction.dot(&oc);
                let c = oc.magnitude_squared() - sphere.radius.powi(2);
                let discriminant = h * h - a * c;
                
                if discriminant < 0.0 {
                    return false;
                }
                
                let mut t = (h - discriminant.sqrt()) / a;
                if !ray_t.surrounds(t) {
                    t = (h + discriminant.sqrt()) / a;
                    if !ray_t.surrounds(t) {
                        return false;
                    }
                }

                
                hit_record.t = t;
                hit_record.p = ray.at(hit_record.t);

                let outward_normal = (hit_record.p - sphere.center) / sphere.radius;
                hit_record.set_face_normal(ray, outward_normal);

                true
            }
        }
    }
}

// Functions related to anything hittable
struct HitRecord {
    p: Vector3<f32>,
    normal: Vector3<f32>,
    t: f32,
    front_face: bool,
    material: sync::Arc<Material>,
}
impl HitRecord {
    fn set_face_normal(&mut self, ray: &Ray, outward_normal: Vector3<f32>) {
        // Normals will always point against the incident ray
        self.front_face = ray.direction.dot(&outward_normal) < 0.0;
        self.normal = if self.front_face { outward_normal } else { -outward_normal };
    }
}

struct Hittable {
    geometry: Geometry,
    material: sync::Arc<Material>,
}
impl Hittable {
    fn hit(&self, ray: &Ray, ray_t: Interval, hit_record: &mut HitRecord) -> bool {
        self.geometry.hit(ray, ray_t, hit_record)
    }
}

struct HittableList {
    objects: Vec<Hittable>
}
impl HittableList {
    fn add(&mut self, object: Hittable) {
        self.objects.push(object);
    }
    fn hit(&self, ray: &Ray, ray_t: Interval, hit_record: &mut HitRecord) -> bool {
        let mut temp_record = HitRecord {
            p: Vector3::<f32>::new(0.0, 0.0, 0.0),
            normal: Vector3::<f32>::new(0.0, 0.0, 0.0),
            t: f32::INFINITY,
            front_face: false,
            material: sync::Arc::new(Material::Blank),
        };

        let mut hit_anything = false;
        let mut closest_so_far = ray_t.max;

        for object in self.objects.iter() {
            if object.hit(ray, Interval {min: ray_t.min, max: closest_so_far}, &mut temp_record) {
                hit_anything = true;
                closest_so_far = temp_record.t;
                hit_record.p = temp_record.p;
                hit_record.normal = temp_record.normal;
                hit_record.t = temp_record.t;
                hit_record.front_face = temp_record.front_face;
                hit_record.material = object.material.clone();
            }
        }

        hit_anything
    }
}

// Functions related to materials
enum Material {
    Blank,
    Lambertian(Lambertian),
    Metal(Metal),
    Dielectric(Dielectric),
}
impl Material {
    fn scatter(&self, ray: &Ray, hit_record: &HitRecord, attenuation: &mut Vector3<f32>, scattered: &mut Ray) -> bool {
        match self {
            Material::Blank => false,
            Material::Lambertian(lambertian) => {
                let mut scatter_direction = hit_record.normal + random_unit_vector();

                if near_zero(&scatter_direction) {
                    scatter_direction = hit_record.normal;
                }

                scattered.origin = hit_record.p;
                scattered.direction = scatter_direction;

                attenuation.x = lambertian.albedo.x;
                attenuation.y = lambertian.albedo.y;
                attenuation.z = lambertian.albedo.z;

                return true;
            },
            Material::Metal(metal) => {
                let reflected = reflect(&ray.direction.normalize(), &hit_record.normal).normalize() + (metal.fuzz * random_unit_vector());

                scattered.origin = hit_record.p;
                scattered.direction = reflected;

                attenuation.x = metal.albedo.x;
                attenuation.y = metal.albedo.y;
                attenuation.z = metal.albedo.z;

                return scattered.direction.dot(&hit_record.normal) > 0.0;
            },
            Material::Dielectric(dielectric) => {
                return false;
            }
        }
    }
}

struct Lambertian {
    albedo: Vector3<f32>,
}
struct Metal {
    albedo: Vector3<f32>,
    fuzz: f32,
}
struct Dielectric {}

// Functions related to intervals
struct Interval {
    min: f32,
    max: f32,
}
impl Interval {
    fn size(&self) -> f32 {
        self.max - self.min
    }
    fn contains(&self, value: f32) -> bool {
        self.min <= value && value <= self.max
    }
    fn surrounds(&self, value: f32) -> bool {
        self.min < value && value < self.max
    }
    fn clamp(&self, value: f32) -> f32 {
        if value < self.min {
            self.min
        } else if value > self.max {
            self.max
        } else {
            value
        }
    }
    fn expand(&self, value: f32) -> Interval {
        let padding = value / 2.0;
        Interval {
            min: self.min - padding,
            max: self.max + padding
        }
    }
}

// Functions related to camera
struct Camera {
    image_width: i32,
    image_height: i32,
    center: Vector3<f32>,
    pixel00_loc: Vector3<f32>,
    pixel_delta_u: Vector3<f32>,
    pixel_delta_v: Vector3<f32>,
    samples_per_pixel: i32,
    pixel_samples_scale: f32,
    max_depth: i32,
    defocus_angle: f32,
    defocus_disk_u: Vector3<f32>,
    defocus_disk_v: Vector3<f32>,
}
impl Camera {
    fn new(aspect_ratio: f32, image_width: i32, samples_per_pixel: i32, max_depth: i32, vfov: i32, lookfrom: Vector3<f32>, lookat: Vector3<f32>, vup: Vector3<f32>, defocus_angle: f32, focus_dist: f32) -> Camera {
        let image_height = cmp::max((image_width as f32 / aspect_ratio) as i32, 1);

        // Calculate the viewport size
        let theta = vfov as f32 * f32::consts::PI / 180.0;
        let h = (theta / 2.0).tan();
        let viewport_height = 2.0 * h * focus_dist;
        let viewport_width = (image_width as f32 / image_height as f32) * viewport_height;

        let w = (lookfrom - lookat).normalize();
        let u = vup.cross(&w).normalize();
        let v = w.cross(&u);

        // Setup the viewport coordinates
        let viewport_u = viewport_width * u;
        let viewport_v = viewport_height * -v;

        let camera_center = lookfrom;

        let pixel_delta_u = viewport_u / image_width as f32;
        let pixel_delta_v = viewport_v / image_height as f32;

        let viewport_upper_left = camera_center - (focus_dist * w) - viewport_u / 2.0 - viewport_v / 2.0;

        let pixel00_loc = viewport_upper_left + (pixel_delta_u + pixel_delta_v) / 2.0;

        let defocus_radius = focus_dist * (defocus_angle / 2.0).to_radians();

        let defocus_disk_u = defocus_radius * u;
        let defocus_disk_v = defocus_radius * v;

        let pixel_samples_scale = 1.0 / samples_per_pixel as f32;

        Camera {
            image_width,
            image_height,
            center: camera_center,
            pixel00_loc,
            pixel_delta_u,
            pixel_delta_v,
            samples_per_pixel,
            pixel_samples_scale,
            max_depth,
            defocus_angle,
            defocus_disk_u,
            defocus_disk_v,
        }
    }
    fn ray_color(ray: &Ray, depth: i32, world: &HittableList) -> Vector3<f32> {
        if depth <= 0 {
            return Vector3::<f32>::new(0.0, 0.0, 0.0);
        }

        let mut hit_record = HitRecord {
            p: Vector3::<f32>::new(0.0, 0.0, 0.0),
            normal: Vector3::<f32>::new(0.0, 0.0, 0.0),
            t: f32::INFINITY,
            front_face: false,
            material: sync::Arc::new(Material::Blank),
        };

        if world.hit(ray, Interval { min: 0.001, max: f32::INFINITY }, &mut hit_record) {
            let mut scattered = Ray { origin: Vector3::<f32>::new(0.0, 0.0, 0.0), direction: Vector3::<f32>::new(0.0, 0.0, 0.0) };
            let mut attenuation = Vector3::<f32>::new(0.0, 0.0, 0.0);

            if hit_record.material.scatter(ray, &hit_record, &mut attenuation, &mut scattered) {
                let new_color = Camera::ray_color(&scattered, depth - 1, world);

                return Vector3::<f32>::new(attenuation.x * new_color.x, attenuation.y * new_color.y, attenuation.z * new_color.z);
            }
            return Vector3::<f32>::new(0.0, 0.0, 0.0);
        }
    
        let unit_direction = ray.direction.normalize();
        let t = 0.5 * (unit_direction.y + 1.0);
        (1.0 - t) * Vector3::<f32>::new(1.0, 1.0, 1.0) + t * Vector3::<f32>::new(0.5, 0.7, 1.0)
    }
    fn sample_square() -> Vector3<f32> {
        Vector3::<f32>::new(rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5, 0.0)
    }
    fn defocus_disk_sample(&self) -> Vector3<f32> {
        let disk_sample = random_in_unit_disk();
        
        self.center + (disk_sample.x * self.defocus_disk_u) + (disk_sample.y * self.defocus_disk_v)
    }
    fn get_ray(&self, i: i32, j: i32) -> Ray {
        let offset = Camera::sample_square();
        let pixel_sample = self.pixel00_loc + (i as f32 + offset.x) * self.pixel_delta_u + (j as f32 + offset.y) * self.pixel_delta_v;
        let ray_origin = if self.defocus_angle <= 0.0 { self.center } else { self.defocus_disk_sample() };
        let ray_direction = pixel_sample - self.center;

        Ray { origin: ray_origin, direction: ray_direction }
    }
    fn render(&self, image_buffer: &mut BufWriter<Vec<u8>>, world: &HittableList, progress_bar: &ProgressBar) -> std::io::Result<()> {
        (0..self.image_height).for_each(|j| {
            (0..self.image_width).for_each(|i| {
                progress_bar.inc(1);
                let mut pixel_color = Vector3::<f32>::new(0.0, 0.0, 0.0);

                pixel_color = (0..self.samples_per_pixel).into_par_iter().map(|_| {
                    let ray = self.get_ray(i, j);
                    Camera::ray_color(&ray, self.max_depth, world)
                }).sum();

                pixel_color *= self.pixel_samples_scale;

                write_color(image_buffer, &pixel_color).unwrap();
            })
        });

        image_buffer.flush()?;
        progress_bar.finish();
        Ok(())
    }
}

// Main execution function
fn main() -> std::io::Result<()> {
    // Create the world
    let mut world = HittableList { objects: Vec::new() };

    /*
    let ground_material = sync::Arc::new(Material::Lambertian(Lambertian { albedo: Vector3::<f32>::new(0.5, 0.5, 0.5) }));

    world.add(Hittable {
        geometry: Geometry::Sphere(Sphere { center: Vector3::<f32>::new(0.0, -1000.0, 0.0), radius: 1000.0 }),
        material: ground_material.clone(),
    });
    */


    let cube_mesh = import_stl_model("./models/cube.stl");
    let model_material = sync::Arc::new(Material::Lambertian( Lambertian { albedo: Vector3::<f32>::new(1.0, 1.0, 1.0) }));

    add_model_to_world(&mut world, cube_mesh, model_material);
    
    /*
    for a in -11..11 {
        for b in -11..11 {
            let choose_mat = rand::random::<f32>();

            let center = Vector3::<f32>::new(a as f32 + 0.9 * rand::random::<f32>(), 0.2, b as f32 + 0.9 * rand::random::<f32>());

            if (center - Vector3::<f32>::new(4.0, 0.2, 0.0)).magnitude() > 0.9 {
                let sphere_material: sync::Arc<Material>;

                if choose_mat < 0.8 {
                    let albedo = Vector3::<f32>::new(rand::random::<f32>() * rand::random::<f32>(), rand::random::<f32>() * rand::random::<f32>(), rand::random::<f32>() * rand::random::<f32>());
                    sphere_material = sync::Arc::new(Material::Lambertian(Lambertian { albedo }));
                } else {
                    let albedo = Vector3::<f32>::new(0.5 * (1.0 + rand::random::<f32>()), 0.5 * (1.0 + rand::random::<f32>()), 0.5 * (1.0 + rand::random::<f32>()));
                    let fuzz = 0.5 * rand::random::<f32>();
                    sphere_material = sync::Arc::new(Material::Metal(Metal { albedo, fuzz }));
                }

                world.add(Hittable {
                    geometry: Geometry::Sphere(Sphere { center, radius: 0.2 }),
                    material: sphere_material,
                });
            }
        }
    }
    
    let material1 = sync::Arc::new(Material::Lambertian(Lambertian { albedo: Vector3::<f32>::new(0.4, 0.2, 0.1) }));
    let material2 = sync::Arc::new(Material::Lambertian(Lambertian { albedo: Vector3::<f32>::new(0.7, 0.6, 0.5) }));
    let material3 = sync::Arc::new(Material::Metal(Metal { albedo: Vector3::<f32>::new(0.7, 0.6, 0.5), fuzz: 0.0 }));

    world.add(Hittable {
        geometry: Geometry::Sphere(Sphere { center: Vector3::<f32>::new(0.0, 1.0, 0.0), radius: 1.0 }),
        material: material1.clone(),
    });
    world.add(Hittable {
        geometry: Geometry::Sphere(Sphere { center: Vector3::<f32>::new(-4.0, 1.0, 0.0), radius: 1.0 }),
        material: material2.clone(),
    });
    world.add(Hittable {
        geometry: Geometry::Sphere(Sphere { center: Vector3::<f32>::new(4.0, 1.0, 0.0), radius: 1.0 }),
        material: material3.clone(),
    });
    */

    // Setup the camera
    let aspect_ratio = 16.0 / 9.0;
    let image_width = 400;
    let samples_per_pixel = 10;
    let max_depth = 10;
    let vfov = 20;
    let lookfrom = Vector3::<f32>::new(13.0, 2.0, 3.0);
    let lookat = Vector3::<f32>::new(0.0, 0.0, 0.0);
    let vup = Vector3::<f32>::new(0.0, 1.0, 0.0);
    let defocus_angle = 0.0;
    let focus_dist = 10.0;

    let camera = Camera::new(aspect_ratio, image_width, samples_per_pixel, max_depth, vfov, lookfrom, lookat, vup, defocus_angle, focus_dist);

    // Setup the progress bar
    let progress_bar = ProgressBar::new((image_width * camera.image_height) as u64);

    // Setup the image buffer
    let mut image_buffer = BufWriter::new(Vec::new());

    // Render the image
    camera.render(&mut image_buffer, &world, &progress_bar)?;

    // Write the image to a file
    let mut file = File::create("image.ppm")?;
    file.write(format!("P3\n{} {}\n255\n", image_width, camera.image_height).as_bytes())?;
    file.write_all(&image_buffer.get_ref())?;
    file.flush()?;

    Ok(())
}
