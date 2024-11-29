#define sensor 2 //
#define led 3 //

  float sine;
  int frequency; 
  int x = 0; 

void setup(){ 
  
  pinMode(sensor, INPUT); 
  pinMode(led, OUTPUT); 
  Serial.begin(9600); 

} 

void loop(){ 
  
  sine = sin(x*3.1416/180); //pi?
  frequency = (int(sine*500));
  if(x>90){ 
    x = 0; 
 }else{
    x++; 
} 
  
  int sensor_value = digitalRead(sensor);
  
  Serial.println( sensor_value);
  if ( sensor_value == 0){ 
    
    digitalWrite(led,HIGH);
    
    digitalWrite(led,LOW); 
    
    
  
 }
  delay(10);
}
