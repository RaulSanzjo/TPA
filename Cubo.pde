

PBDSystem crea_cubo(float alto,
    float ancho,
    float largo,
    float dens,
    int n_alto,
    int n_ancho,
    int n_largo,
    float stiffness,
    float display_size){
   
  int N = n_alto*n_ancho*n_largo;
  float masa = dens*alto*ancho*largo;
  PBDSystem cubo = new PBDSystem(N,masa/N);
  
  float dx = ancho/(n_ancho-1.0);
  float dy = alto/(n_alto-1.0);
  float dz = largo/(n_largo-1.0);

  int id = 0;
  for (int i = 0; i< n_ancho;i++){
    for(int j = 0; j< n_alto;j++){
      for(int k = 0; k< n_largo; k++){
        Particle p = cubo.particles.get(id);
        p.location.set(dx*i,dy*j,dz*k);
        p.display_size = display_size;
  
        id++;
      }
    }
  }
  
  /**
   * Creo restricciones de distancia. Aquí sólo se crean restricciones de estructura.
   * Faltarían las de shear y las de bending.
   */
  id = 0;
  for (int i = 0; i < n_ancho; i++){
    for(int j = 0; j < n_alto; j++){
      for(int k = 0; k < n_largo; k++){
        println("id: "+id+" (i,j,k) = ("+i+","+j+","+k+")");
        Particle p = cubo.particles.get(id);
        if(i > 0){
          int idx = id - n_ancho * n_largo;
          Particle px = cubo.particles.get(idx);
          Constraint c = new DistanceConstraint(p,px,dx,stiffness);        
          cubo.add_constraint(c);
          println("Restricción creada: "+ id+"->"+idx);
        }
  
        if(j > 0){
          int idy = id - n_largo;
          Particle py = cubo.particles.get(idy);
          Constraint c = new DistanceConstraint(p,py,dy,stiffness);
          cubo.add_constraint(c);
          println("Restricción creada: "+ id+"->"+idy);
        }
        
        if(k > 0){
          int idz = id - 1;
          Particle pz = cubo.particles.get(idz);
          Constraint c = new DistanceConstraint(p,pz,dz,stiffness);
          cubo.add_constraint(c);
          println("Restricción creada: "+ id+"->"+idz);
        }
        
        if(i < n_ancho - 1 && j < n_alto - 1){
          // Triángulo esquina inferior izquierda
          int id_v = id + n_ancho * n_largo + n_largo;
          Particle v = cubo.particles.get(id_v);
          
          int id_b1 = id + n_largo;
          Particle b1 = cubo.particles.get(id_b1);
          
          PVector vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), v.location), 3);   
          float h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t1 = new TriangleConstraint(p,b1,v,h0,stiffness);        
          cubo.add_constraint(t1);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_v);
          
          // Triángulo esquina superior derecha 
          id_v = id + n_largo;
          v = cubo.particles.get(id_v);
          
          vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), v.location), 3);   
          h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t2 = new TriangleConstraint(p,b1,v,h0,stiffness);        
          cubo.add_constraint(t2);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_v);
          
          // Triángulo esquina superior izquirda
          int id_b0 = id + n_largo * n_ancho;
          Particle b0 = cubo.particles.get(id_b0);
          
          id_b1 = id + n_largo;
          b1 = cubo.particles.get(id_b1);
          
          vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), b0.location), 3);   
          h0 = PVector.sub(p.location, vec_c).mag();      
          
          Constraint t3 = new TriangleConstraint(b0,b1,p,h0,stiffness);        
          cubo.add_constraint(t3);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_b0);
          
          // Triángulo esquina inferior derecha        
          id_b1 = id + n_largo;
          b1 = cubo.particles.get(id_b1);
          
          id_v = id + n_largo + n_largo * n_ancho;
          v = cubo.particles.get(id_v);
          
          vec_c = PVector.div(PVector.add(PVector.add(b0.location, b1.location), v.location), 3);   
          h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t4 = new TriangleConstraint(b0,b1,v,h0,stiffness);        
          cubo.add_constraint(t4);
          //println("Restricción creada: "+ id_v+"->"+id_b1+"->"+id_b0);
        }
        
        if(k < n_largo - 1 && i < n_ancho - 1){
          // Triángulo esquina inferior izquierda
          int id_v = id + n_ancho * n_largo + 1;
          Particle v = cubo.particles.get(id_v);
          
          int id_b1 = id + 1;
          Particle b1 = cubo.particles.get(id_b1);
          
          PVector vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), v.location), 3);   
          float h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t1 = new TriangleConstraint(p,b1,v,h0,stiffness);        
          cubo.add_constraint(t1);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_v);
          
          // Triángulo esquina superior derecha 
          id_v = id + 1;
          v = cubo.particles.get(id_v);
          
          vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), v.location), 3);   
          h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t2 = new TriangleConstraint(p,b1,v,h0,stiffness);        
          cubo.add_constraint(t2);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_v);
          
          // Triángulo esquina superior izquirda
          int id_b0 = id + n_largo * n_ancho;
          Particle b0 = cubo.particles.get(id_b0);
          
          id_b1 = id + 1;
          b1 = cubo.particles.get(id_b1);
          
          vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), b0.location), 3);   
          h0 = PVector.sub(p.location, vec_c).mag();      
          
          Constraint t3 = new TriangleConstraint(b0,b1,p,h0,stiffness);        
          cubo.add_constraint(t3);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_b0);
          
          // Triángulo esquina inferior derecha        
          id_b1 = id + 1;
          b1 = cubo.particles.get(id_b1);
          
          id_v = id + 1 + n_largo * n_ancho;
          v = cubo.particles.get(id_v);
          
          vec_c = PVector.div(PVector.add(PVector.add(b0.location, b1.location), v.location), 3);   
          h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t4 = new TriangleConstraint(b0,b1,v,h0,stiffness);        
          cubo.add_constraint(t4);
          //println("Restricción creada: "+ id_v+"->"+id_b1+"->"+id_b0);
        }
        
         if(k < n_largo - 1 && j < n_alto - 1){
          // Triángulo esquina inferior izquierda
          int id_v = id + n_ancho + 1;
          Particle v = cubo.particles.get(id_v);
          
          int id_b1 = id + 1;
          Particle b1 = cubo.particles.get(id_b1);
          
          PVector vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), v.location), 3);   
          float h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t1 = new TriangleConstraint(p,b1,v,h0,stiffness);        
          cubo.add_constraint(t1);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_v);
          
          // Triángulo esquina superior derecha 
          id_v = id + 1;
          v = cubo.particles.get(id_v);
          
          vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), v.location), 3);   
          h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t2 = new TriangleConstraint(p,b1,v,h0,stiffness);        
          cubo.add_constraint(t2);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_v);
          
          // Triángulo esquina superior izquirda
          int id_b0 = id + n_ancho;
          Particle b0 = cubo.particles.get(id_b0);
          
          id_b1 = id + 1;
          b1 = cubo.particles.get(id_b1);
          
          vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), b0.location), 3);   
          h0 = PVector.sub(p.location, vec_c).mag();      
          
          Constraint t3 = new TriangleConstraint(b0,b1,p,h0,stiffness);        
          cubo.add_constraint(t3);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_b0);
          
          // Triángulo esquina inferior derecha        
          id_b1 = id + 1;
          b1 = cubo.particles.get(id_b1);
          
          id_v = id + 1 +  n_ancho;
          v = cubo.particles.get(id_v);
          
          vec_c = PVector.div(PVector.add(PVector.add(b0.location, b1.location), v.location), 3);   
          h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t4 = new TriangleConstraint(b0,b1,v,h0,stiffness);        
          cubo.add_constraint(t4);
          //println("Restricción creada: "+ id_v+"->"+id_b1+"->"+id_b0);
        }
  
        id++;
      }
    }
  }
  
  // Fijamos cuatro esquinas
  id = (n_alto * n_ancho) - n_largo;
  cubo.particles.get(id).set_bloqueada(true); 
  
  id = n_alto * n_ancho - 1;
  cubo.particles.get(id).set_bloqueada(true);
  
  id = N - n_alto;
  cubo.particles.get(id).set_bloqueada(true);
  
  id = N - 1;
  cubo.particles.get(id).set_bloqueada(true); 
  
  print("Cubo creado con " + cubo.particles.size() + " partículas y " + cubo.constraints.size() + " restricciones."); 
  
  return cubo;
}
