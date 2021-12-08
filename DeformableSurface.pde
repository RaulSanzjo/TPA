import java.util.TreeMap;

public class DeformableSurface 
{
  float _lengthX;   // Length of the surface in X direction (m)
  float _lengthY;   // Length of the surface in Y direction (m)
  
  int _numNodesX;   // Number of nodes in X direction
  int _numNodesY;   // Number of nides in Y direction

  SpringLayout _springLayout;  // Physical layout of the springs that define the surface
  boolean _isUnbreakable;   // True if the surface cannot be broken
  color _color;   // Color (RGB)

  Particle[][] _nodes;   // Particles defining the surface
  ArrayList<DampedSpring> _springsSurface;   // Springs joining the particles
  TreeMap<String, DampedSpring> _springsCollision;   // Springs for collision handling


  DeformableSurface(float lengthX, float lengthY, int numNodesX, int numNodesY, float surfacePosZ, float nodeMass, float Ke, float Kd, float maxForce, float breakLengthFactor, SpringLayout springLayout, boolean isUnbreakable, color c)
  {
    _lengthX = lengthX;
    _lengthY = lengthY;

    _numNodesX = numNodesX;
    _numNodesY = numNodesY;
    
    _springLayout = springLayout;
    _isUnbreakable = isUnbreakable;
    _color = c;

    _nodes = new Particle[_numNodesX][_numNodesY];
    _springsSurface = new ArrayList();
    _springsCollision = new TreeMap<String, DampedSpring>();

    createNodes(surfacePosZ, nodeMass);
    createSurfaceSprings(Ke, Kd, maxForce, breakLengthFactor);
  }

  void createNodes(float surfacePosZ, float nodeMass)
  {
    /* Este método debe dar valores al vector de nodos ('_nodes') en función del
       tamaño que tenga la malla y de las propiedades de ésta (número de nodos, 
       masa de los mismos, etc.).
     */
     for(int i = 0; i <  _numNodesX; i++){
       for(int j = 0; j < _numNodesY; j++){
         if(i == 0 || j == 0 || i == (_numNodesX - 1) || j == (_numNodesY - 1)){
           _nodes[i][j] = new Particle(new PVector((i * (_lengthX / _numNodesX) - _lengthX/2), (j * (_lengthY / _numNodesY) - _lengthY/2), surfacePosZ), new PVector(0,0,0), nodeMass);    
           _nodes[i][j].set_bloqueada(true);
         }
         else{
           _nodes[i][j] = new Particle(new PVector((i * (_lengthX / _numNodesX) - _lengthX/2), (j * (_lengthY / _numNodesY) - _lengthY/2), surfacePosZ), new PVector(0,0,0), nodeMass);
          _nodes[i][j].set_bloqueada(false);
         }
       }
     }     
  }

  void createSurfaceSprings(float Ke, float Kd, float maxForce, float breakLengthFactor)
  {
    /* Este método debe añadir muelles a la lista de muelles de la malla ('_springsSurfaces')
       en función de la disposición deseada para éstos dentro de la malla, y de los parámetros
       de los muelles (Ke, Kd, etc.).
     */
     if(_springLayout == SpringLayout.STRUCTURAL){
       for(int i = 0; i <  _numNodesX; i++){
         for(int j = 0; j < _numNodesY; j++){
           if(i < _numNodesX - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j], Ke, Kd, false, maxForce, breakLengthFactor));
           if(j < _numNodesY - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
         }   
       }
     }
     else if(_springLayout == SpringLayout.SHEAR){
       for(int i = 0; i <  _numNodesX; i++){
         for(int j = 0; j < _numNodesY - 1; j++){
           if(i > 0 && i < _numNodesX - 1){
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i-1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
           }
           else if(i == 0)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
           else if(i == _numNodesX - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i-1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
         }   
       }
     }
     else if(_springLayout == SpringLayout.STRUCTURAL_AND_SHEAR){
       for(int i = 0; i <  _numNodesX; i++){
         for(int j = 0; j < _numNodesY; j++){
           if(i < _numNodesX - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j], Ke, Kd, false, maxForce, breakLengthFactor));
           if(j < _numNodesY - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
         }   
       }
       for(int i = 0; i <  _numNodesX; i++){
         for(int j = 0; j < _numNodesY - 1; j++){
           if(i > 0 && i < _numNodesX - 1){
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i-1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
           }
           else if(i == 0)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
           else if(i == _numNodesX - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i-1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
         }   
       }
     }
     else if(_springLayout == SpringLayout.STRUCTURAL_AND_BEND){
       for(int i = 0; i <  _numNodesX; i++){
         for(int j = 0; j < _numNodesY; j++){
           if(i < _numNodesX - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j], Ke, Kd, false, maxForce, breakLengthFactor));
           if(j < _numNodesY - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
           if(i < _numNodesX - 2)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+2][j], Ke, Kd, false, maxForce, breakLengthFactor));
           if(j < _numNodesY - 2)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i][j+2], Ke, Kd, false, maxForce, breakLengthFactor));
         }   
       }
     }
     else if(_springLayout == SpringLayout.STRUCTURAL_AND_SHEAR_AND_BEND){
       for(int i = 0; i <  _numNodesX; i++){
         for(int j = 0; j < _numNodesY; j++){
           if(i < _numNodesX - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j], Ke, Kd, false, maxForce, breakLengthFactor));
           if(j < _numNodesY - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
           if(i < _numNodesX - 2)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+2][j], Ke, Kd, false, maxForce, breakLengthFactor));
           if(j < _numNodesY - 2)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i][j+2], Ke, Kd, false, maxForce, breakLengthFactor));
         }   
       }
       for(int i = 0; i <  _numNodesX; i++){
         for(int j = 0; j < _numNodesY - 1; j++){
           if(i > 0 && i < _numNodesX - 1){
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i-1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
           }
           else if(i == 0)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i+1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
           else if(i == _numNodesX - 1)
             _springsSurface.add(new DampedSpring(_nodes[i][j], _nodes[i-1][j+1], Ke, Kd, false, maxForce, breakLengthFactor));
         }   
       }
     }
  }

  void update(float simStep)
  {
    int i, j;

    for (i = 0; i < _numNodesX; i++)
      for (j = 0; j < _numNodesY; j++)
        if (_nodes[i][j] != null)
          _nodes[i][j].update(simStep);

    for (DampedSpring s : _springsSurface) 
    {
      s.update(simStep);
      s.applyForces();
    }

    for (DampedSpring s : _springsCollision.values()) 
    {
      s.update(simStep);
      s.applyForces();
    }
  }

  void draw()
  {
    if (_isUnbreakable) 
       drawWithQuads();
    else
       drawWithSegments();
  }

  void drawWithQuads()
  {
    int i, j;
    
    fill(255);
    stroke(_color);

    for (j = 0; j < _numNodesY - 1; j++)
    {
      beginShape(QUAD_STRIP);
      for (i = 0; i < _numNodesX; i++)
      {
        if ((_nodes[i][j] != null) && (_nodes[i][j+1] != null))
        {
          PVector pos1 = _nodes[i][j].getLocation();
          PVector pos2 = _nodes[i][j+1].getLocation();

          vertex(pos1.x, pos1.y, pos1.z);
          vertex(pos2.x, pos2.y, pos2.z);
        }
      }
      endShape();
    }
  }

  void drawWithSegments()
  {
    stroke(_color);

    for (DampedSpring s : _springsSurface) 
    {
      if (!s.isBroken())
      {
        PVector pos1 = s.getParticle1().getLocation();
        PVector pos2 = s.getParticle2().getLocation();

        line(pos1.x, pos1.y, pos1.z, pos2.x, pos2.y, pos2.z);
      }
    }
  }
}
