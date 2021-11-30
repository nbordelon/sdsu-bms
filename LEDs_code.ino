void setup() {
  pinMode(11, OUTPUT);
  pinMode(12,  OUTPUT);  
  pinMode(13, OUTPUT);  
  pinMode(14, OUTPUT);    
  pinMode(15, OUTPUT);
}
void loop() {
  float packVolt = 12.8;
  while(packVolt >= 0){
  if(packVolt<= 12.8*0.2){
  //Turn the red light on when charge is between %0 - %20
  digitalWrite(11, HIGH); //Red light 
  //Turn off the rest
  digitalWrite(12, LOW);   
  digitalWrite(13, LOW); 
  digitalWrite(14, LOW); 
  digitalWrite(15, LOW); 
 // delay(1000); //Do we need a delay if we are not using a button?
  }
  else if(packVolt > 12.8 * 0.2 && packVolt <= 12.8 *0.4){
    //Turn on red and the first green LED and turn off the rest
     digitalWrite(11, HIGH); 
     digitalWrite(12, HIGH; 
    digitalWrite(13, LOW); 
    digitalWrite(14, LOW); 
    digitalWrite(15, LOW); 
     // delay(1000);
  } else if(packVolt > 12.8 * 0.6 && packVolt <= 12.8 * 0.8 ){
    //Turn on red and the two first green LED and turn off the rest
      digitalWrite(11, HIGH); 
      digitalWrite(12, HIGH; 
      digitalWrite(13, HIGH); 
      digitalWrite(14, LOW); 
      digitalWrite(15, LOW); 
   // delay(1000);
  } else{
     digitalWrite(11, HIGH); 
     digitalWrite(12, HIGH);   
     digitalWrite(13, HIGH); 
     digitalWrite(14, HIGH); 
     digitalWrite(15, HIGH);
     // delay(1000); 
  }

}         
}
