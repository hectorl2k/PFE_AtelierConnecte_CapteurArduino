#include <elapsedMillis.h>
#include <Ethernet.h>
#include <SPI.h>
#include <SharpIR.h>

//////////////// Génrérale ////////////////////////////////////
String ID_Boitier="Soutenance";
String urlScript= "/PFE_AtelierConnecte_CapteurWeb/BDD/add.php";
#define delay_EnvoiServ 500

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
    const byte PinCapteurVibra[Nbr_CaptDist_Vibra]= {A5} ;        // pin capteur vibration

    #define delay_VibraGetRes 500                         // Temps durant lequel on calul la vibration moyenne
    #define delay_PeriodVibra 20                          // Temps durant lequel on calul le vibration

    elapsedMillis timer_getResVibra;                       // Timer maj vibration
    elapsedMillis timer_getNewDataVibra;                       // Timer Get vibration


    /*  Fonction Utilisée  Vibration */

    bool setupVibra(void);            // Setup Capteur de Vibration
    float GetVibra(byte pin_Vibra);    // Reucuperer en %
    void gestionVibration();

//////////////// Capteur Distance ////////////////////////////////////

    #define delay_PeriodDist 50                      // toutes les x on recupere la distance
    #define model 1080
    
       /////     MODE FLUX    /////
    #define FonctionnementModeFlux true                  // Si aucun capteur mettre false sion true
    #define delay_FluxGetRes 5000                         // Temps durant lequel on calul le flux
    #define Nbr_CaptDist_Flux 1                           // Nombre de capteur en mode flux
    const int PinCapteurDistFluxSharp[Nbr_CaptDist_Flux][2]= {{A0,1080}} ;  // Pin + model 
    const float DistDetectObject[Nbr_CaptDist_Flux]={200};        // Distance en mm pour detecter un objet
    elapsedMillis timer_getResFlux;                       // Timer maj flux
    elapsedMillis timer_getNewDataFlux;                       // Timer Get distance

       /////  MODE DISTANCE MOY   /////
    #define FonctionnementModeDist true                  // Si aucun capteur mettre false sion true
    #define delay_DistGetRes 500                      // Temps durant lequel on calul la distance moyenne
    #define Nbr_CaptDist_Dist 1
    const int PinCapteurDistDistSharp[Nbr_CaptDist_Dist][2]= {{A0,1080}} ;  

    
    elapsedMillis timer_getResDist;                       // Timer Get distance
    elapsedMillis timer_getNewDataDist;                       // Timer Get distance



    /*  Fonction Utilisée  Distance */
   void Setup_Sharp();
   float getDist_Sharp();
   void gestionDist_Flux();
   void gestionDist_Dist();








   /////////////////////////////////////////////////////////////////////////////////////////////
   //////////////////////////  Ne pas modifier    //////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////////////////////////


  int i_nbr_mesure_Vibra[Nbr_CaptDist_Vibra]={0};       // Nombre de mesure
  float tempResVibra[Nbr_CaptDist_Vibra]={0};           // addition des mesures
  float ResVibra[Nbr_CaptDist_Vibra]={0};               // Variable resultat vibration moyenne


  bool detectObject[Nbr_CaptDist_Flux] ={false};

  int tempResFlux[Nbr_CaptDist_Flux]={0};             // Variable Temp result flux
  float ResFlux[Nbr_CaptDist_Flux]={0};                    // Variable resultat flux

  int i_nbr_mesure_dist[Nbr_CaptDist_Dist]={0};       // Nombre de mesure
  float tempResDist[Nbr_CaptDist_Dist]={0};  // addition des mesures
  float ResDist[Nbr_CaptDist_Dist]={0};                    // Variable resultat distance moyenne
  String urlDist="";
  String urlFlux="";
  String urlVibra="";
  bool FirstRequete=true;



void setup() {
  Serial.begin(115200);
  SetupEthernet();
  SetupVibra();
  Setup_Sharp();
  timer_getResFlux=0;    // Reset TIMER


}


void loop() {
  if( FonctionnementVibra == true)
    {gestionVibration();}
  if( FonctionnementModeFlux == true)
    {gestionDist_Flux();}
  if( FonctionnementModeDist == true)
    {gestionDist_Dist();}
  EnvoiServeur();
}




bool SetupVibra(void)
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


     // Serial.print("Fin de calcul de la vibration Moyenne, nb itération : ");
     // Serial.print(i_nbr_mesure_Vibra[i]+1);
     // Serial.print("Vibration moyenne : ");
     // Serial.println(ResVibra[i]);

      tempResVibra[i]=0;
      i_nbr_mesure_Vibra[i]=0;

    }
   }}





float getDist_Sharp (int pinCapt[])
{   int ir_val[25] = {};
    int distanceCM;
    float current;
    int median;

   for (int i=0; i<25; i++){
        // Read analog value
        ir_val[i] = analogRead(pinCapt[0]); 
    }

    for(int i=0; i<(25-1); i++) {
        bool flag = true;
        for(int o=0; o<(25-(i+1)); o++) {
            if(ir_val[o] > ir_val[o+1]) {
                int t = ir_val[o];
                ir_val[o] = ir_val[o+1];
                ir_val[o+1] = t;
                flag = false;  
            }
        }
    }
    median = ir_val[25/2]; 
    
    if (pinCapt[1]==1080) {
        distanceCM = 29.988 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.173);
    }
     return (distanceCM*10.0);
}





void Setup_Sharp()
{
  if(FonctionnementModeFlux == true)
  {
    for(int i=0;i<Nbr_CaptDist_Flux;i++)
    {
      pinMode (PinCapteurDistFluxSharp[i][0], INPUT);
    }
  }

  if(FonctionnementModeDist == true)
  {
    for(int i=0;i<Nbr_CaptDist_Dist;i++)
    {
        pinMode (PinCapteurDistDistSharp[i][0], INPUT);
    }
  }  
}




void gestionDist_Flux()
{
  float resDist;

   if (timer_getNewDataFlux > delay_PeriodDist)
   {
    timer_getNewDataFlux=0;       //Reset Timer
    for(int i=0;i<Nbr_CaptDist_Flux;i++)
    {
        resDist=  getDist_Sharp(PinCapteurDistFluxSharp[i]);      // Reucpere la distance

      

      if( resDist < DistDetectObject[i] && detectObject[i]==false)     // regarde si on detcte un nouvel objet
      {
        detectObject[i]=true;
        Serial.println("NEW OBJECT");
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
     // Serial.println(ResFlux[i]);

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
      resDist=  getDist_Sharp(PinCapteurDistDistSharp[i]);      // Reucpere la distance
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

      // Serial.print("Fin de calcul de la Disatnce Moyenne mm, nb itération : ");
      // Serial.print(i_nbr_mesure_dist[i]);
      // Serial.print("  Mesure  moyenne en mm : ");
      // Serial.println(ResDist[i]+1);

      tempResDist[i]=0;
      i_nbr_mesure_dist[i]=0;
      //Serial.println(ResDist[i]);
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

void EnvoiServeur()
{
  if (FirstRequete ==true && timer_sendData >= (delay_EnvoiServ*2))
  {
    FirstRequete =false;
  }

  if(timer_sendData >= delay_EnvoiServ && FirstRequete ==false)
  {
      timer_sendData=0;
        if (client.connect(server, port)) {
          Serial.println("Boitier Connecté au serveur");


        // Serial.println("connected - Try to send Data");
         //Serial.println("Data Send");     //YOUR URL
         String url="GET " + urlScript + "?ID_Boitier=" + ID_Boitier + urlVibra + urlDist + urlFlux ;
         //Serial.println("URL : " +url);
         //Serial.println("\n\n");
         client.println(url);     //YOUR URL
         client.println(" HTTP/1.1");
         client.println("Host: 192.168.2.5");
         client.println("Connection: close");
         client.println();
        } else {
    String url="GET " + urlScript + "?ID_Boitier=" + ID_Boitier + urlVibra + urlDist + urlFlux ;
   // Serial.println(url);
    Serial.println("connection failed");
    
  }
  }
}
