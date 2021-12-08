

PBDSystem crea_tela(float alto,
    float ancho,
    float dens,
    int n_alto,
    int n_ancho,
    float stiffness,
    float display_size){
   
  int N = n_alto*n_ancho;
  float masa = dens*alto*ancho;
  PBDSystem tela = new PBDSystem(N,masa/N);
  
  float dx = ancho/(n_ancho-1.0);
  float dy = alto/(n_alto-1.0);
  
  int id = 0;
  for (int i = 0; i< n_ancho;i++){
    for(int j = 0; j< n_alto;j++){
      Particle p = tela.particles.get(id);
      p.location.set(dx*i,dy*j,0);
      p.display_size = display_size;

      id++;
    }
  }
  
  /**
   * Creo restricciones de distancia. Aquí sólo se crean restricciones de estructura.
   * Faltarían las de shear y las de bending.
   */
  id = 0;
  for (int i = 0; i< n_ancho;i++){
    for(int j = 0; j< n_alto;j++){
      //println("id: "+id+" (i,j) = ("+i+","+j+")");
      Particle p = tela.particles.get(id);
      if(i>0){
        int idx = id - n_alto;
        Particle px = tela.particles.get(idx);
        Constraint c = new DistanceConstraint(p,px,dx,stiffness);        
        tela.add_constraint(c);
        //println("Restricción creada: "+ id+"->"+idx);
      }

      if(j>0){
        int idy = id - 1;
        Particle py = tela.particles.get(idy);
        Constraint c = new DistanceConstraint(p,py,dy,stiffness);
        tela.add_constraint(c);
        //println("Restricción creada: "+ id+"->"+idy);
      }
      
      if(type_structure == 0){
        if(i < n_ancho - 1 && j < n_alto - 1){
          // Triángulo esquina inferior izquierda
          int id_v = id + n_alto;
          Particle v = tela.particles.get(id_v);
          
          int id_b1 = id + n_alto + 1;
          Particle b1 = tela.particles.get(id_b1);
          
          PVector vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), v.location), 3);   
          float h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t1 = new TriangleConstraint(p,b1,v,h0,stiffness);        
          tela.add_constraint(t1);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_v);
          
          // Triángulo esquina superior derecha 
          id_v = id + 1;
          v = tela.particles.get(id_v);
          
          vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), v.location), 3);   
          h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t2 = new TriangleConstraint(p,b1,v,h0,stiffness);        
          tela.add_constraint(t2);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_v);
          
          // Triángulo esquina superior izquirda
          int id_b0 = id + n_alto;
          Particle b0 = tela.particles.get(id_b0);
          
          id_b1 = id + 1;
          b1 = tela.particles.get(id_b1);
          
          vec_c = PVector.div(PVector.add(PVector.add(p.location, b1.location), b0.location), 3);   
          h0 = PVector.sub(p.location, vec_c).mag();      
          
          Constraint t3 = new TriangleConstraint(b0,b1,p,h0,stiffness);        
          tela.add_constraint(t3);
          //println("Restricción creada: "+ id+"->"+id_b1+"->"+id_b0);
          
          // Triángulo esquina inferior derecha        
          id_b1 = id + 1;
          b1 = tela.particles.get(id_b1);
          
          id_v = id + 1 + n_alto;
          v = tela.particles.get(id_v);
          
          vec_c = PVector.div(PVector.add(PVector.add(b0.location, b1.location), v.location), 3);   
          h0 = PVector.sub(v.location, vec_c).mag();      
          
          Constraint t4 = new TriangleConstraint(b0,b1,v,h0,stiffness);        
          tela.add_constraint(t4);
          //println("Restricción creada: "+ id_v+"->"+id_b1+"->"+id_b0);
        }
      }
      if(type_structure == 3){
        if(i < n_ancho - 1 && j < n_alto - 1){
          int idx = id + n_ancho + 1;
          Particle px = tela.particles.get(idx);
          Constraint c1 = new DistanceConstraint(p,px,dx,stiffness);        
          tela.add_constraint(c1);
          //println("Restricción creada: "+ id+"->"+idx);  
        }
        if(j > 0 && i < n_ancho -1){
          int idy = id + n_alto - 1;
          Particle py = tela.particles.get(idy);
          Constraint c1 = new DistanceConstraint(p,py,dy,stiffness);        
          tela.add_constraint(c1);
          //println("Restricción creada: "+ id+"->"+idx);  
        }
      }


      id++;
    }
  }
  
  // Fijamos dos esquinas
  id = n_alto-1;
  tela.particles.get(id).set_bloqueada(true); 
  
  id = N-1;
  tela.particles.get(id).set_bloqueada(true); 
  
  print("Tela creada con " + tela.particles.size() + " partículas y " + tela.constraints.size() + " restricciones."); 
  println("");

  return tela;
}
