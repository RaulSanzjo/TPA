
abstract class Constraint{

  ArrayList<Particle> particles;
  float stiffness;    // k en el paper de Muller
  float k_coef;       // k' en el paper de Muller
  float C;
  
  Constraint(){
    particles = new ArrayList<Particle>();
  }
  
  void  compute_k_coef(int n){
    k_coef = 1.0 - pow((1.0-stiffness),1.0/float(n));
    //println("Fijamos "+n+" iteraciones   -->  k = "+stiffness+"    k' = "+k_coef+".");
  }

  abstract void proyecta_restriccion();
  abstract void display(float scale_px);
}

class DistanceConstraint extends Constraint{

  float d;
  
  DistanceConstraint(Particle p1,Particle p2,float dist,float k){
    super();
    d = dist;
    particles.add(p1);
    particles.add(p2);
    stiffness = k;
    k_coef = stiffness;
    C=0;

  }
  
  void proyecta_restriccion(){
    Particle part1 = particles.get(0); 
    Particle part2 = particles.get(1);
    
    PVector p1 = part1.location;
    PVector p2 = part2.location;
    
    float w1 = part1.w;
    float w2 = part2.w;
    
    PVector vec_d = PVector.sub(p1, p2);

    if(debug){
      println("PROYECTA: p1=" + p1);
      println("PROYECTA: p2=" + p2);
      println("PROYECTA: p1-p2=" + vec_d);
    }

    float restriccion = vec_d.mag() - d;
    PVector n = vec_d.div(vec_d.mag()); 
    PVector delta = PVector.mult(n, restriccion / (w1 + w2));
    
    part1.location.add(PVector.mult(delta, -w1 * this.k_coef));
    part2.location.add(PVector.mult(delta, w2 * this.k_coef));
  }
  
  void display(float scale_px){
    PVector p1 = particles.get(0).location; 
    PVector p2 = particles.get(1).location; 
    strokeWeight(3);
    stroke(255,255*(1-abs(4*C/d)),255*(1-abs(4*C/d)));
    line(scale_px*p1.x, -scale_px*p1.y, scale_px*p1.z,  scale_px*p2.x, -scale_px*p2.y, scale_px*p2.z);
  };
  
}

class TriangleConstraint extends Constraint{
  PVector b0, b1, v;
  PVector c;
  float h0;

  TriangleConstraint(Particle p1,Particle p2,Particle p3,float dist,float k){
    super();
    particles.add(p1);
    particles.add(p2);
    particles.add(p3);
    h0 = dist;

    stiffness = k;
    k_coef = stiffness;
    C=0;
  }
  
  void proyecta_restriccion(){
    Particle part1 = particles.get(0); 
    Particle part2 = particles.get(1);
    Particle part3 = particles.get(2);
    
    b0 = part1.location;
    b1 = part2.location;
    v = part3.location;
    
    if(debug){
      print("b0: " + b0);
      print("    b1: " + b1);
      println("    v: " + v);
    }

    PVector b0_b1 = PVector.add(b0, b1);
    PVector b0_b1_v = PVector.add(b0_b1, v);
    c = PVector.div(b0_b1_v, 3);
    
    PVector vec_h0 = PVector.sub(v, c);
    if(debug){
      print("vec_h0: " + vec_h0);
    }

    float vec_h0_mag = vec_h0.mag();
    float W = part1.w + part2.w + 2 * part3.w;
    
    if(vec_h0_mag != 0 && W != 0){
      float unit = (1 - h0 / vec_h0_mag);
      PVector delta = PVector.mult(vec_h0, unit / W * k_coef);
      PVector delta_b0 = PVector.mult(delta, 2 * part1.w);
      PVector delta_b1 = PVector.mult(delta, 2 * part2.w);
      PVector delta_v = PVector.mult(delta, -4 * part3.w);
      
      if(debug){
        print("        delta_b0: " + delta_b0);
        print("        delta_b1: " + delta_b1);
        println("        delta_v: " + delta_v);
      }

      part1.location.add(delta_b0);
      part2.location.add(delta_b1);
      part3.location.add(delta_v);
    }
  }
  
  void display(float scale_px){
    PVector p1 = particles.get(0).location; 
    PVector p2 = particles.get(1).location; 
    PVector p3 = particles.get(2).location; 

    strokeWeight(3);
    stroke(255,255*(1-abs(4*C/h0)),255*(1-abs(4*C/h0)));
    line(scale_px*p1.x, -scale_px*p1.y, scale_px*p1.z,  scale_px*p2.x, -scale_px*p2.y, scale_px*p2.z);
    line(scale_px*p1.x, -scale_px*p1.y, scale_px*p1.z,  scale_px*p3.x, -scale_px*p3.y, scale_px*p3.z);
    line(scale_px*p2.x, -scale_px*p2.y, scale_px*p2.z,  scale_px*p3.x, -scale_px*p3.y, scale_px*p3.z);
  };
}
