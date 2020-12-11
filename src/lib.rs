use wasm_bindgen::prelude::*;
use nalgebra::{
    Vector3 
  };
// When the `wee_alloc` feature is enabled, this uses `wee_alloc` as the global
// allocator.
//
// If you don't want to use `wee_alloc`, you can safely delete this.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

  #[wasm_bindgen]
  #[derive(Copy, Clone, Debug, PartialEq)]
  pub struct Vect3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
  }
  #[wasm_bindgen]
  impl Vect3 {
    pub fn new(x:f64,y:f64,z:f64) -> Vect3 {
      Vect3 { x,y,z}
    }
  }


  #[wasm_bindgen]
  pub struct Ray {
    origin: Vector3<f64>,
    dir: Vector3<f64>,
  }
  #[wasm_bindgen]
  impl Ray {
    #[wasm_bindgen(constructor)]
    pub fn new(x0:f64,y0:f64,z0:f64,x:f64,y:f64,z:f64) -> Ray {
        let origin = Vector3::new(x0,y0,z0);
        let dir = Vector3::new(x,y,z);
        Ray { origin,dir}
    }

    pub fn intersects(&self ,radius:f64) -> Vec<f64> {
        let r2 = radius*radius;
        let origin2 = self.origin.dot(&self.origin);
        let origin_dir = self.origin.dot(&self.dir);
        let dir2 = self.dir.dot(&self.dir);
        let a = dir2;
        let b = 2.0*origin_dir;
        let c = origin2 - r2;
        resolve_quad(a, b,c);
        return vec![]
    }
  }



  #[wasm_bindgen]
  pub struct Plane {
    origin: Vector3<f64>,
    dir1: Vector3<f64>,
    dir2: Vector3<f64>,
  }
  //   impl Plane {
  //     static mut WGS84: Vector3<f64> =  Vector3::new(6378137.0, 6378137.0, 6356752.3142451793);
  // }
  #[wasm_bindgen]
  impl Plane {
    #[wasm_bindgen(constructor)]
    pub fn new(x:f64,y:f64,z:f64,x1:f64,y1:f64,z1:f64,x2:f64,y2:f64,z2:f64) -> Plane {
      let origin = Vector3::new(x,y,z);
      let dir1 = Vector3::new(x1,y1,z1);
      let dir2 = Vector3::new(x2,y2,z2);
      Plane {
        origin,
        dir1,
        dir2
      }
    }
    pub fn tangents(&self , radius:f64) -> Vec<f64> {
      let tangents = tangents_sphere_full(self.origin,self.dir1,self.dir2,radius);
      // let tangent0 = *tangents.get(0).unwrap();
      let mut vec : Vec<f64> = vec![];
      for tang in tangents {
          vec.push(tang.x);
          vec.push(tang.y);
          vec.push(tang.z);
      }
      return vec
    }
  }
  // compute plane tangents with sphere
  pub fn tangents_sphere(origin:Vector3<f64>, dir_0:Vector3<f64>, dir_1:Vector3<f64>, radius:f64) -> Vec<Vect3> {
    let r2 = radius*radius;
    let origin2 = origin.dot(&origin);
    // compute alpha
    let dir1_origin = dir_1.dot(&origin);
    let dir1_2 = dir_1.dot(&dir_1);
    let dir0_2 = dir_0.dot(&dir_0);
    let dir1_dir0 = dir_0.dot(&dir_1);
    let dir0_origin = dir_0.dot(&origin);
    let a = dir1_origin*dir1_origin - dir1_2 * (origin2-r2);
    let b = 2.0*dir1_origin*dir0_origin - 2.0*dir1_dir0 * (origin2 -r2);
    let c = dir0_origin*dir0_origin - (origin2-r2)*dir0_2;
    let alphas = resolve_quad(a, b,c);
    // let see the alphas:
    // 1 plane tangent
    // 2 the 2 tangent with the sphere
    // 0 plane does not cross the sphere
    let mut vec : Vec<Vect3> = vec![];
  
    for alpha in &alphas {
      // compute the t for the final vector3
      let a = dir0_2 + alpha * alpha * dir1_2  + 2.0 * alpha * dir1_dir0;
      let b = 2.0 * (dir0_origin +alpha * dir1_origin);
      let t = -b / (2.0*a);
      // compute the resulting vector
      let x = origin + (dir_0 +  dir_1.scale(*alpha)).scale(t);
      let x_conv = Vect3{
        x : x.x,
        y : x.y,
        z : x.z
      };
      vec.push(x_conv);
    }
    return vec
  }
  
  /** 
   * compute plane tangents with sphere . prepare the vector for more robust configuration 
   **/
  pub fn tangents_sphere_full(origin:Vector3<f64>, dir_0:Vector3<f64>, dir_1:Vector3<f64>, radius:f64) -> Vec<Vect3> {
    // plane normal
    let normal = dir_0.cross(&dir_1);
    let norm_normal = normal.normalize();
    let closest = norm_normal.scale(norm_normal.dot(&origin));
    // direction to closest
    let point_to_closest = closest - origin;
    let point_to_closest_norm = point_to_closest.try_normalize(0.0);
    let point_to_closest_0 = point_to_closest_norm.unwrap_or_else(|| dir_0);
    // ortonogal direction
    let second_direction = norm_normal.cross(&point_to_closest_0).normalize();
    return tangents_sphere(origin,point_to_closest_0,second_direction,radius)
  }
  
  /**
   * resolve quadratic function
   * */
  pub fn resolve_quad(a:f64,b:f64,c:f64)->Vec<f64>{
    let delta = b*b - 4.0 * a* c;
    if a ==0.0 {
      if  c == 0.0 {
        let x = 0.0;
        return vec![x];
      }
       let x = -b/c;
       return vec![x];
    }
    if delta == 0.0 {
        let x = -b / (2.0 * a);
        return vec![x];
    } else if  delta > 0.0 {
        let delta_sqrt = delta.sqrt();
        let x1 = (delta_sqrt-b) / (2.0 * a);
        let x2 = (-delta_sqrt-b) / (2.0 * a);
        return vec![x1, x2];
    }
    // no solutions in real plane
    return vec![];
  }
  
  
  #[cfg(test)]
  mod tests {
    use crate::Vect3;
    use nalgebra::Vector3;
    use crate::tangents_sphere_full;
    use wasm_bindgen_test::*;
  
  wasm_bindgen_test_configure!(run_in_browser);
    
      #[test]
      fn it_works_for_tangent() {
          let v1 = Vector3::new(1.0, 0.0,0.0);
          let v2 = Vector3::new(0.0, 0.0,1.0);
          let v3 = Vector3::new(0.0, 1.0,0.0);
          let tangents = tangents_sphere_full(v1,v2,v3,1.0);
          assert_eq!(tangents.len(), 1);
          assert_eq!(tangents[0], Vect3::new(1.0, 0.0,0.0));
      }
      #[test]
      fn it_works_for_tangen2t() {
          let v1 = Vector3::new(1.0, 1.0,0.0);
          let v2 = Vector3::new(0.0, 0.0,1.0);
          let v3 = Vector3::new(0.0, 1.0,0.0);
          let tangents = tangents_sphere_full(v1,v2,v3,1.0);
          assert_eq!(tangents.len(), 1);
          assert_eq!(tangents[0], Vect3::new(1.0, 0.0,0.0));
      }
      #[test]
      fn it_works_for_tangen3t() {
          let v1 = Vector3::new(1.0, 1.0,1.0);
          let v2 = Vector3::new(0.0, 0.0,1.0);
          let v3 = Vector3::new(0.0, 1.0,0.0);
          let tangents = tangents_sphere_full(v1,v2,v3,1.0);
          assert_eq!(tangents.len(), 1);
          assert_eq!(tangents[0], Vect3::new(1.0, 0.0,0.0));
      }
      #[test]
      fn it_works_for_tangen4t() {
          let v1 = Vector3::new(1.0, 1.0,0.0);
          let v2 = Vector3::new(0.0, 1.0,0.0);
          let v3 = Vector3::new(0.0, 0.0,1.0);
          let tangents = tangents_sphere_full(v1,v2,v3,1.0);
          assert_eq!(tangents.len(), 1);
          assert_eq!(tangents[0], Vect3::new(1.0, 0.0,0.0));
      }
      #[test]
      fn it_works_for_non_crossing() {
        let v1 = Vector3::new(2.0, 0.0,0.0);
        let v2 = Vector3::new(0.0, 0.0,1.0);
        let v3 = Vector3::new(0.0, 1.0,0.0);
        let tangents = tangents_sphere_full(v1,v2,v3,1.0);
        assert_eq!(tangents.len(), 0);
    }
    #[test]
    fn it_works_for_crossing() {
      let v1 = Vector3::new(0.98, 1.0,0.0);
      let v2 = Vector3::new(0.0, 0.0,1.0);
      let v3 = Vector3::new(0.0, 1.0,0.0);
      let tangents = tangents_sphere_full(v1,v2,v3,1.0);
      assert_eq!(tangents.len(), 2);
  }
  }