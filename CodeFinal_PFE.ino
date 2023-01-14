#include <elapsedMillis.h>
#include <Ethernet.h>
#include <SPI.h>

//////////////// Génrérale ////////////////////////////////////
String ID_Boitier="ROMA0";
String urlScript= "/PFE_AtelierConnecte_CapteurWeb/BDD/add.php";
#define delay_EnvoiServ 5000

byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x9E, 0x1E };
byte ip[] = { 192, 168, 2, 5 };
byte server[] = { 192, 168, 2, 1 }; // Touchberry Pi Server
int port = 1800;
EthernetClient client;
elapsedMillis timer_sendData;


 /*  Fonction Utilisée  Internet */  
void SetupEthernet();

//////////////// Capteur Vibration ////////////////////////////////////                

    #define FonctionnementVibra true                  // Si aucun capteur mettre false sinon true 
    #define Nbr_CaptDist_Vibra 1                           // Nombre de capteur de vibration
    const byte PinCapteurVibra[Nbr_CaptDist_Vibra]= {A1} ;        // pin capteur vibration

    #define delay_VibraGetRes 5000                         // Temps durant lequel on calul la vibration moyenne
    #define delay_PeriodVibra 100                          // Temps durant lequel on calul le vibration 
  
    elapsedMillis timer_getResVibra;                       // Timer maj vibration
    elapsedMillis timer_getNewDataVibra;                       // Timer Get vibration


    /*  Fonction Utilisée  Vibration */  
    
    bool setupVibra(void);            // Setup Capteur de Vibration
    float GetVibra(byte pin_Vibra);    // Reucuperer en % 
    void gestionVibration();  

//////////////// Capteur Distance ////////////////////////////////////

    #define delay_PeriodDist 100                      // toutes les x on recupere la distance

    
       /////     MODE FLUX    /////
    #define FonctionnementModeFlux true                  // Si aucun capteur mettre false sion true 
    #define delay_FluxGetRes 5000                         // Temps durant lequel on calul le flux
    #define Nbr_CaptDist_Flux 1                           // Nombre de capteur en mode flux
    const byte PinCapteurDistFlux[Nbr_CaptDist_Flux][2]= {{2,3}} ;        // nbr Capteur de flux, ( pin triger, pin echo)
    const float DistDetectObject[Nbr_CaptDist_Flux]={100};        // Distance en mm pour detecter un objet  
    elapsedMillis timer_getResFlux;                       // Timer maj flux
    elapsedMillis timer_getNewDataFlux;                       // Timer Get distance

       /////  MODE DISTANCE MOY   /////
    #define FonctionnementModeDist true                  // Si aucun capteur mettre false sion true 
    #define delay_DistGetRes 5000                      // Temps durant lequel on calul la distance moyenne
    #define Nbr_CaptDist_Dist 2 
    const byte PinCapteurDistDist[Nbr_CaptDist_Dist][2]= {{2,3}} ;     // nbr Capteur de flux, ( pin triger, pin echo)
    elapsedMillis timer_getResDist;                       // Timer Get distance
    elapsedMillis timer_getNewDataDist;                       // Timer Get distance
    

   
    /*  Fonction Utilisée  Distance */  
    
    float getDist_HCSR04(byte pinCapt[]);
    void setup_HCSR04();
    void gestionDist_Flux(); 
    void gestionDist_Dist();
    






   /////////////////////////////////////////////////////////////////////////////////////////////
   //////////////////////////  Ne pas modifier    //////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////////////////////////


  int i_nbr_mesure_Vibra[Nbr_CaptDist_Vibra]={0};       // Nombre de mesure
  float tempResVibra[Nbr_CaptDist_Vibra]={0};           // addition des mesures
  float ResVibra[Nbr_CaptDist_Vibra]={0};               // Variable resultat vibration moyenne 


  /*  Const HC-SR04   */ 
  const unsigned long MEASURE_TIMEOUT = 25000UL; /* const Timout : 25ms = ~8m à 340m/s  */
  const float SOUND_SPEED = 340.0 / 1000;         /* Vitesse du son dans l'air en mm/us */
  #define pin_Trigger 0
  #define pin_Echo 1
  bool detectObject[Nbr_CaptDist_Flux] ={false};
  
  int tempResFlux[Nbr_CaptDist_Flux]={0};             // Variable Temp result flux
  float ResFlux[Nbr_CaptDist_Flux]={0};                    // Variable resultat flux 

  int i_nbr_mesure_dist[Nbr_CaptDist_Dist]={0};       // Nombre de mesure
  float tempResDist[Nbr_CaptDist_Dist]={0};  // addition des mesures
  float ResDist[Nbr_CaptDist_Dist]={0};                    // Variable resultat distance moyenne 
  String urlDist="";
  String urlFlux="";
  String urlVibra="";
  
  


  

void setup() {
  Serial.begin(115200); 
  SetupEthernet(); 
  setupVibra();
  setup_HCSR04();
  
  
  timer_getResFlux=0;    // Reset TIMER 

}

void loop() {

   if( FonctionnementVibra == true)
  {
    gestionVibration();
  }  
  if( FonctionnementModeFlux == true)
  {
    gestionDist_Flux();
  }

 if( FonctionnementModeDist == true)
  {
    gestionDist_Dist();
  }
SendData();

}






bool setupVibra(void)
{
  if(FonctionnementVibra==true)
  {
     for(int i=0;i<Nbr_CaptDist_Vibra;i++)
     {

      pinMode(PinCapteurVibra[i],INPUT);
     }
  }
  return true;
}




float getVibra(byte pin_Vibra)
{
  float res= ((analogRead(pin_Vibra)*100.0)/1023);
  return res;
}


void gestionVibration()
{
  float resVibra;
   

   if (timer_getNewDataVibra > delay_PeriodVibra)
   {
    timer_getNewDataVibra=0;       //Reset Timer
    for(int i=0;i<Nbr_CaptDist_Vibra;i++)
    {
      resVibra=  getVibra(PinCapteurVibra[i]);      // Reucpere la vibration
      tempResVibra[i] += resVibra;
      i_nbr_mesure_Vibra[i]++;     
    }
   }
    if (timer_getResVibra > delay_VibraGetRes)
   {
    timer_getResVibra=0;       //Reset Timer
     urlVibra="&nbrCaptVibration="+String(Nbr_CaptDist_Vibra);     // prepare les info a envoyer au serveur
     for(int i=0;i<Nbr_CaptDist_Vibra;i++)
    {
      ResVibra[i]=(tempResVibra[i]*1.0)/i_nbr_mesure_Vibra[i];
      urlVibra+="&Vibration"+String(i)+"="+String(ResVibra[i]);

      tempResVibra[i]=0;
      i_nbr_mesure_Vibra[i]=0;
      Serial.println(ResVibra[i]);
    }
   }}

void setup_HCSR04()
{ 
  if(FonctionnementModeFlux == true)
  {
    for(int i=0;i<Nbr_CaptDist_Flux;i++)
    {
          /* Initialise les broches */
      pinMode(PinCapteurDistFlux[i][pin_Trigger], OUTPUT);
      digitalWrite(PinCapteurDistFlux[i][pin_Trigger], LOW); // La broche TRIGGER doit être à LOW au repos
      pinMode(PinCapteurDistFlux[i][pin_Echo], INPUT);
    }
  }

  if(FonctionnementModeDist == true)
  {
    for(int i=0;i<Nbr_CaptDist_Dist;i++)
    {
          /* Initialise les broches */
      pinMode(PinCapteurDistDist[i][pin_Trigger], OUTPUT);
      digitalWrite(PinCapteurDistDist[i][pin_Trigger], LOW); // La broche TRIGGER doit être à LOW au repos
      pinMode(PinCapteurDistDist[i][pin_Echo], INPUT);
    }
  }
}
float getDist_HCSR04(byte pinCapt[])
{
  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(pinCapt[pin_Trigger], HIGH);
  delayMicroseconds(10);
  digitalWrite(pinCapt[pin_Trigger], LOW);
  
  /* 2. Mesure le temps echo l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure = pulseIn(pinCapt[pin_Echo], HIGH, MEASURE_TIMEOUT);
   
  /* 3. Calcul la distance à partir du temps mesuré */
  float distance_mm = measure / 2.0 * SOUND_SPEED;
  delay(10);
  return distance_mm;
}






void gestionDist_Flux()
{
  float resDist;
 
   if (timer_getNewDataFlux > delay_PeriodDist)
   {
    timer_getNewDataFlux=0;       //Reset Timer
    for(int i=0;i<Nbr_CaptDist_Flux;i++)
    {
      resDist=  getDist_HCSR04(PinCapteurDistFlux[i]);      // Reucpere la distance
      
      if( resDist < DistDetectObject[i] && detectObject[i]==false)     // regarde si on detcte un nouvel objet 
      {
        detectObject[i]=true;
         tempResFlux[i]++;
      }


      if(resDist >DistDetectObject[i] && detectObject[i]==true)       // Fin de la detection de l'objet;
      {
        detectObject[i]=false;
      }  
    }
   }

    if (timer_getResFlux > delay_FluxGetRes)
   {
    timer_getResFlux=0;       //Reset Timer
    urlFlux="&nbrDistance_Flux="+String(Nbr_CaptDist_Flux);     // prepare les info a envoyer au serveur
     for(int i=0;i<Nbr_CaptDist_Flux;i++)
    {
      
      ResFlux[i]=tempResFlux[i]/(delay_FluxGetRes/1000.0);    // Calcul nbr Objet Seconde
      urlFlux+="&Distance_Flux"+String(i)+"="+String(ResFlux[i]);
      tempResFlux[i]=0;
      Serial.println(ResFlux[i]);
      
    }
   }  
}





void gestionDist_Dist()
{
  float resDist;
 
   if (timer_getNewDataDist > delay_PeriodDist)
   {
    timer_getNewDataDist=0;       //Reset Timer
    for(int i=0;i<Nbr_CaptDist_Dist;i++)
    {
      resDist=  getDist_HCSR04(PinCapteurDistDist[i]);      // Reucpere la distance
      tempResDist[i] += resDist;
      i_nbr_mesure_dist[i]++;     
    }
   }
    if (timer_getResDist > delay_DistGetRes)
   {
    timer_getResDist=0;       //Reset Timer
    urlDist="&nbrDistance_Dist="+String(Nbr_CaptDist_Dist);     // prepare les info a envoyer au serveur
     for(int i=0;i<Nbr_CaptDist_Dist;i++)
    {
      ResDist[i]=(tempResDist[i]*1.0)/i_nbr_mesure_dist[i];
      urlDist+="&Distance_Dist"+String(i)+"="+String(ResDist[i]);

      tempResDist[i]=0;
      i_nbr_mesure_dist[i]=0;
      Serial.println(ResDist[i]);   
    }
   }  
  
}


void SetupEthernet()
{

  
  if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
  Ethernet.begin(mac, ip);
  delay(3000);
  Serial.println(Ethernet.localIP());

}

void SendData()
{
  if(timer_sendData >= delay_EnvoiServ)
  {
      timer_sendData=0;
        if (client.connect(server, port)) {
         Serial.println("connected - Try to send Data");
         Serial.print("Data Send");     //YOUR URL      
         String url="GET " + urlScript + "?ID_Boitier=" + ID_Boitier + urlVibra + urlDist + urlFlux ;
       
         client.println(url);     //YOUR URL
         client.println(" HTTP/1.1");
         client.println("Host: 192.168.2.5");
         client.println("Connection: close");
         client.println();
        } else {
    
    Serial.println("connection failed");
  }
  }
}
   
