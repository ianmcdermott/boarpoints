import shiffman.box2d.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;

Box2DProcessing box2d;


float[] grav ={-50, -100, -250, -500, -1000, -2500, -5000};

ArrayList<Circle> cir;


Boundary boundary;
float ran = random(1);

float[] coordX, coordY;
int coordCount = 0;

void setup(){
  size(800, 800);
  String[] lines = loadStrings("coords.tsv");
  
  coordCount = lines.length;
  coordX = new float[coordCount];
  coordY = new float[coordCount];
 
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  box2d.setGravity(grav[int(random(6.999))], 0);

  cir = new ArrayList<Circle>();
  for(int i = 0; i < coordCount; i++){
    float[] pieces = float(split(lines[i],", "));
    coordX[i] = pieces[0];
    coordY[i] = pieces[1];
    
 //   boundary = new Boundary(new Vec2(coordX[i], coordY[i]));
   
  }
   background(255);
}

void draw(){
  box2d.step();
 // background(255);
   for(int i = 0; i < 20; i++){
       Circle ci = new Circle(width, random(height), random(.5,3), random(-5,-1));{
      cir.add(ci);
    }
   }
/*  Circle cirr = new Circle(width, random(height), random(.5,3), random(-5,-1));{
    cir.add(cirr);
  }
  Circle cirrc = new Circle(width, random(height), random(.5,3), random(-5,-1));{
    cir.add(cirrc);
  }*/
  
  for (Circle c: cir){
  // for(int i = 0; i < coordCount; i++){
 //   Vec2 force = boundary.attract(c);
 //   c.applyForce(force);
      c.display();
      
    }
  //  }
  
  
  for(int i = cir.size()-1; i >=0; i --){
    Circle c = cir.get(i);
    if(c.done()){
      cir.remove(i);
    }
  }
}

class Boundary{
  float r;
  float skin = 1;
  Body body;
  Vec2 offset =  new Vec2(width/2-512,  height/2-384);
   Boundary(Vec2 xy){
     Vec2 pos = new Vec2();
     pos = xy.add(offset);
     
     BodyDef bd = new BodyDef();
     bd.type = BodyType.STATIC;
        bd.position.set(box2d.coordPixelsToWorld(pos));
       body = box2d.createBody(bd);
     
     
     CircleShape cs = new CircleShape();
     cs.m_radius = box2d.scalarPixelsToWorld(r/2);
     
     FixtureDef fd = new FixtureDef();
     fd.shape = cs;
     fd.density = 0;
     fd.friction = 0;
     fd.restitution = 0;
     
     body.createFixture(fd);
     
 
  }
  
  void display(){
    Vec2 pos = box2d.getBodyPixelCoord(body);

    pushMatrix();
    translate(pos.x,pos.y);
    strokeWeight(3);
    stroke(255, 0, 0);
    point(0,0);
    popMatrix();
        
  }
  
  Vec2 attract(Circle c){
    Vec2 pos = body.getWorldCenter();
    Vec2 cirPos = c.body.getWorldCenter();
    Vec2 force = pos.sub(cirPos);
    float dist = force.length();
    dist = constrain(dist, 0, 20);
   force.normalize();
   float strength = (skin)/(dist*dist) ;
   force.mulLocal(strength);
   return force;
  }
}
class Circle{
  float x, y, off, r, alph;
  Body body;
  int alive = 450;
  
  Circle(float x_, float y_, float r_, float off_){
    x = x_;
    y = y_;
    r = 1;
    off = off_;
    
 
    
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(box2d.coordPixelsToWorld(x,y));
    body = box2d.createBody(bd);
    
    CircleShape cs = new CircleShape();
    cs.m_radius = box2d.scalarPixelsToWorld(r/2);
    
    FixtureDef fd = new FixtureDef();
    fd.shape = cs;
    fd.density = 0;
    fd.friction = .5;
    fd.restitution = 0;
    
    body.createFixture(fd);
    body.setLinearVelocity(new Vec2(off, 2));
  }
  
  void display(){
    Vec2 pos = box2d.getBodyPixelCoord(body);
    float a = body.getAngle()*-1;
    
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(a);
    fill(0);
    noStroke();
    ellipse(0, 0, r, r);
    popMatrix();
  }
  
  void applyForce(Vec2 f){
    Vec2 pos = body.getWorldCenter();
    body.applyForce(f, pos);
  }
  
  void killBody(){
    box2d.destroyBody(body);
  }
  
  void update(){
    alive--;
  }
  
  boolean done() {
    // Let's find the screen position of the particle
    Vec2 pos = box2d.getBodyPixelCoord(body);
    // Is it off the bottom of the screen?
    if (alive <= 0) {
      killBody();
      return true;
      }
      return false;
      }     
}



