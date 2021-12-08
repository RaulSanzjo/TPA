import peasy.*;

// Display stuff:

final boolean FULL_SCREEN = false;
final int DRAW_FREQ = 50;   // Draw frequency (Hz or Frame-per-second)
int DISPLAY_SIZE_X = 1000;   // Display width (pixels)
int DISPLAY_SIZE_Y = 1000;   // Display height (pixels)

final float FOV = 60;   // Field of view (º)
final float NEAR = 0.01;   // Camera near distance (m)
final float FAR = 100000.0;   // Camera far distance (m)

final color NET_COLOR = color(255, 255, 255);   // Net lines color (RGB)
final color BACKGROUND_COLOR = color(20, 20, 55);   // Background color (RGB)

PeasyCam cam;
float scale_px = 1000;

boolean debug = false;
int type_structure = 0; // 0 -> tela, 1 -> cubo, 2 -> masa-muelle

PBDSystem system;

float dt = 1.0/60.0;
int n_iters = 60;

final PVector G = new PVector(0.0, 0.0, -9.81);   // Acceleration due to gravity (m/(s*s))

// Propiedades del viento 
PVector vel_viento= new PVector(0,0,0);
float viento; //modulo de la intensidad del viento

// Propiedades tela
float ancho_tela = 0.5;
float alto_tela = 0.5;
int n_ancho_tela = 10;
int n_alto_tela = 10;
float densidad_tela = 0.1; // kg/m^2 Podría ser tela gruesa de algodón, 100g/m^2
float sphere_size_tela = ancho_tela/n_ancho_tela*0.4;
float stiffness_tela = 0.95;

// Propiedades cubo
float ancho_cubo = 0.5;
float alto_cubo = 0.5;
float largo_cubo = 0.5;
int n_ancho_cubo = 5;
int n_alto_cubo = 5;
int n_largo_cubo = 5;
float densidad_cubo = 0.1; // kg/m^2 Podría ser tela gruesa de algodón, 100g/m^2
float sphere_size_cubo = ancho_cubo/n_ancho_cubo*0.4;
float stiffness_cubo = 0.95;

// Problem parameters:

// Spring Layout
enum SpringLayout 
{
  STRUCTURAL, 
  SHEAR, 
  STRUCTURAL_AND_SHEAR, 
  STRUCTURAL_AND_BEND, 
  STRUCTURAL_AND_SHEAR_AND_BEND
}

// Simulation values:

final boolean REAL_TIME = true;
final float TIME_ACCEL = 1.0;   // To simulate faster (or slower) than real-time

final float NET_POS_Z = -10.0;   // Position of the net in the Z axis (m)
final int NET_NUMBER_OF_NODES_X = 10;   // Number of nodes of the net in the X direction
final int NET_NUMBER_OF_NODES_Y = 10;   // Number of nodes of the net in the Y direction
final float NET_NODE_MASS = 0.1;   // Mass of the nodes of the net (kg)

final float NET_KE = 60.0;   // Ellastic constant of the net's springs (N/m) 
final float NET_KD = 5.0;   // Damping constant of the net's springs (kg/m)
final float NET_MAX_FORCE = 500.0;   // Maximum force allowed for the net's springs (N)
final float NET_BREAK_LENGTH_FACTOR = 25.0;   // Maximum distance factor (measured in number of times the rest length) allowed for the net's springs

boolean NET_IS_UNBREAKABLE = false;   // True if the net cannot be broken
SpringLayout NET_SPRING_LAYOUT;   // Current spring layout

// Time control:
int _lastTimeDraw = 0;   // Last measure of time in draw() function (ms)
float _deltaTimeDraw = 0.0;   // Time between draw() calls (s)
float _simTime = 0.0;   // Simulated time (s)
float _elapsedTime = 0.0;   // Elapsed (real) time (s)

// Simulated entities:
DeformableSurface _net;   // Deformable mesh

void settings()
{
  if (FULL_SCREEN)
  {
    fullScreen(P3D);
    DISPLAY_SIZE_X = displayWidth;
    DISPLAY_SIZE_Y = displayHeight;
  } 
  else
  {
    size(DISPLAY_SIZE_X, DISPLAY_SIZE_Y, P3D);
  }
}

void initSimulation(SpringLayout springLayout)
{
  _simTime = 0.0;
  _elapsedTime = 0.0;
  NET_SPRING_LAYOUT = springLayout;

  _net = new DeformableSurface(n_ancho_tela, n_alto_tela, NET_NUMBER_OF_NODES_X, NET_NUMBER_OF_NODES_Y, NET_POS_Z, NET_NODE_MASS, NET_KE, NET_KD, NET_MAX_FORCE, NET_BREAK_LENGTH_FACTOR, NET_SPRING_LAYOUT, NET_IS_UNBREAKABLE, NET_COLOR);
}

void updateSimulation()
{
  _net.update(dt);
  _simTime += dt;
}

void setup(){
  println("********************************");
  
  // OJO!! Se le cambia el signo a la y, porque los px aumentan hacia abajo
  if(type_structure == 0 || type_structure == 3){
    cam = new PeasyCam(this, scale_px);
    cam.pan(0.5 * ancho_tela * scale_px, - 0.5 * alto_tela * scale_px);
  
    system = crea_tela(alto_tela,
                      ancho_tela,
                      densidad_tela,
                      n_alto_tela,
                      n_ancho_tela,
                      stiffness_tela,
                      sphere_size_tela);
  }else if(type_structure == 1){       
    cam = new PeasyCam(this, 1.5 * scale_px);
    cam.pan(0.5 * ancho_cubo * scale_px, - 0.5 * alto_cubo *scale_px);
    
    system = crea_cubo(alto_cubo,
                      ancho_cubo,
                      largo_cubo,
                      densidad_cubo,
                      n_alto_cubo,
                      n_ancho_cubo,
                      n_largo_cubo,
                      stiffness_cubo,
                      sphere_size_cubo);
  }
  else{
    float aspect = float(DISPLAY_SIZE_X)/float(DISPLAY_SIZE_Y);  
    perspective((FOV*PI)/180, aspect, NEAR, FAR);
    cam = new PeasyCam(this, 0);
    initSimulation(SpringLayout.STRUCTURAL_AND_SHEAR);
  }
   
  cam.setMinimumDistance(1);
  system.set_n_iters(n_iters);
}

void aplica_viento(){
  if(type_structure != 3){   
     // Aplicamos una fuerza que es proporcional al área.
    // No calculamos la normal. Se deja como ejercicio
    // El área se calcula como el área total, entre el número de partículas
    int npart = system.particles.size();
    float area_total = 0, area;
    if(type_structure == 0)
      area_total = ancho_tela * alto_tela;
    else if(type_structure == 1)
      area_total = ancho_cubo * alto_cubo * largo_cubo;
      
    area = area_total / npart;
    
    for(int i = 0; i < npart; i++){
      float x = (0.5 + random(0.5))*vel_viento.x * area;
      float y = (0.5 + random(0.5))*vel_viento.y * area;
      float z = (0.5 + random(0.5))*vel_viento.z * area;
      PVector fv = new PVector(x,y,z); 
      system.particles.get(i).force.add(fv);
    }
  }
}

void draw(){
  background(BACKGROUND_COLOR);
  lights();
  
  if(type_structure != 2){
    system.apply_gravity(new PVector(0.0,-0.81,0.0));
    aplica_viento();
  
    system.run(dt);  
    
    display();
  }else{
    int now = millis();
    _deltaTimeDraw = (now - _lastTimeDraw)/1000.0;
    _elapsedTime += _deltaTimeDraw;
    _lastTimeDraw = now;
  
    _net.draw();
  
    if (REAL_TIME)
    {
      float expectedSimulatedTime = TIME_ACCEL*_deltaTimeDraw;
      float expectedIterations = expectedSimulatedTime/dt;
      int iterations = 0; 
  
      for (; iterations < floor(expectedIterations); iterations++)
        updateSimulation();
  
      if ((expectedIterations - iterations) > random(0.0, 1.0))
      {
        updateSimulation();
        iterations++;
      }
    } 
    else
      updateSimulation();
  }

  stats();
}

void stats(){  
  pushMatrix();
  {
    camera();
    fill(255);
    textSize(20);
    int npart = system.particles.size();

    text("Frame rate = " + int(frameRate) + " fps", width*0.025, height*0.05);
    //escribe en la pantalla el numero de particulas actuales 
    if(type_structure == 2)
      text("Nº de partículas = " + _net._numNodesX * _net._numNodesY, width*0.025, height*0.075);
    else
      text("Nº de partículas = " + npart, width*0.025, height*0.075);

    if(type_structure == 2){
      if (NET_IS_UNBREAKABLE)
        text("Net is unbreakable", width*0.025, height*0.1);
      else   
        text("Net is breakable", width*0.025, height*0.1);
    }else{
      text("dt = " + dt, width*0.025, height*0.1);
      if(type_structure != 1)
        text("stiffness = " + stiffness_tela, width*0.025, height*0.125);
      else
        text("stiffness = " + stiffness_cubo, width*0.025, height*0.125);
      text("Iteraciones = " + n_iters, width*0.025, height*0.15);
    }
    //--->lo mismo se puede indicar para el viento
    text("Vel. Viento=("+vel_viento.x+", "+vel_viento.y+", "+vel_viento.z+")", width*0.3, height*0.05);
    text("Y - Y negativo / y - Y positivo", width*0.3, height*0.075);
    text("X - X negativo / x - X positivo", width*0.3, height*0.1);
    text("Z - Z negativo / z - Z positivo", width*0.3, height*0.125);
    
    // Otros controles
    text("Stiffness: S: -0.05 / s: +0.05", width*0.65, height*0.075);
    text("Iteraciones: I: -5 / i: +5", width*0.65, height*0.1);
    text("dt: D: -0.01 / d: +0.01", width*0.65, height*0.125);

  }
  popMatrix();
}

void display(){
  int npart = system.particles.size();
  int nconst = system.constraints.size();

  for(int i = 0; i < npart; i++){
    system.particles.get(i).display(scale_px);
  }
  
  for(int i = 0; i < nconst; i++)
      system.constraints.get(i).display(scale_px);    
}

void keyPressed()
{
 // Viento
  if(key == 'Y')
    vel_viento.y -= 0.001;
  if(key == 'y')
    vel_viento.y += 0.001;
  if(key == 'Z')
    vel_viento.z -= 0.001;
  if(key == 'z')
    vel_viento.z += 0.001;
  if(key == 'X')
    vel_viento.x -= 0.001;
  if(key == 'x')
    vel_viento.x += 0.001;
    
  if(key == 'S'){
    if(type_structure == 1 && stiffness_cubo > 0)
      stiffness_cubo -= 0.05;
    else if(stiffness_tela > 0)
      stiffness_tela -= 0.05;
    setup();
  }
  
  if(key == 's'){
    if(type_structure == 1 && stiffness_cubo < 1)
      stiffness_cubo += 0.05;
    else if(stiffness_tela < 1)
      stiffness_tela += 0.05;
    setup();
  }
  
  if(key == 'I'){
    if(n_iters > 0)
        n_iters -= 5;
    setup();
  }
  
  if(key == 'i'){
    n_iters += 5;
    setup();
  }
  
  if(key == 'D'){
    if(dt > 0)
        dt -= 0.01;
    setup();
  }
  
  if(key == 'd'){
    dt += 0.01;
    setup();
  }

    
  // Cambiar entre tela, cubo o masa-muelle
  if(key == '1'){
    type_structure = 0;
    setup();
  }
  if(key == '2'){
    type_structure = 1;
    setup();
  }
  if(key == '3'){
    type_structure = 2;
    setup();
  }
  if(key == '4'){
    type_structure = 3;
    setup();
  }
}  
