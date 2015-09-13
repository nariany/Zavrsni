#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstdlib>
#include <ctime>                                     //Ukljucene biblioteke
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <math.h>
#include <fstream>

//#include <queue>
using namespace std;
//ofstream myfile;
//ofstream drugifile;
//ofstream trecifile;
//std::vector<float> vektorDevijacija1;
//std::vector<float> vektorDevijacija2;
//std::vector<float> vektorDevijacija3;
//Dodatni vektori devijacija
//std::vector<float> vektorDevijacija4;
//std::vector<float> vektorDevijacija5;
//std::vector<float> vektorDevijacija6;
float Max(std::vector<float> Tacke) {
    float maxa=-36415445;
    //Radi kao sat za poztivne vrijednosti,za y=x ne radi i za negativne vrijednosti

    for(int i=0; i<Tacke.size(); i++) {

        if(Tacke[i]>maxa)maxa=Tacke[i];


    }


    return maxa;
}

float Min(std::vector<float> Tacke) {
    float mina=36415445;
    for(int i=0; i<Tacke.size(); i++) {

        if(Tacke[i]<mina)mina=Tacke[i];


    }

    return mina;
}



int myrandom(int i) {
    return std::rand()%i;
}
std::vector<int> randperm(int number) {            //Funkcija koja realizira randperm funkciju iz MATLAB-a
    //std::srand ( unsigned ( std::time(0) ) );
    std::vector<int> pomocniVektor;
    for(int i=1; i<=number; i++)pomocniVektor.push_back(i);
    std::random_shuffle(pomocniVektor.begin(),pomocniVektor.end());
    std::random_shuffle(pomocniVektor.begin(),pomocniVektor.end(),myrandom);
    int povratna_vrijednost=pomocniVektor.size();
    int vrijednost1=rand()%povratna_vrijednost;//Sekvenca za slučajno odabiranje vrijednosti
    int vrijednost2=rand()%povratna_vrijednost;
    std::vector<int>povratniVektor;//Vektor koji vraćamo iz funkcije
    int broj1,broj2;
    broj1=pomocniVektor[vrijednost1];//Trpamo u integere vrijednosti sa slucajno generisanih pozicija
    broj2=pomocniVektor[vrijednost2];
    povratniVektor.push_back(broj1);
    povratniVektor.push_back(broj2);//Poštoje line fitting problem u 2D vraćaju se samo 2 vrijednosti
    return povratniVektor;           //Problem je jedino što nekada vraća iste vrijednosti,a to nije dozvoljeno,pitati DInka kako to izbjeci.

}

//Funkcija koja realizira  distance = normVector*(data - repmat(sample(:,1),1,number)); sekvencu MATLAB koda

std::vector<float> DistanceFunction(std::vector<std::vector<float> >ulazneTacke,std::vector<float> normVector,std::vector<std::vector<float> >sample)   {
//Ovdje je greška.Pazi ovo dobro
    std::vector<std::vector<float> >dataVektor;
    dataVektor.resize(2);
    for(int i=0; i<ulazneTacke[0].size(); i++) {
        float x=ulazneTacke[0][i];
        float y=ulazneTacke[1][i]; //0 i 1 jer imamo samo x i y tacke
        float novax=x-sample[0][0];   //sample[0][0] i sample[1][0] jer uzimamo samo elemente 1 kolone
        float novay=y-sample[1][0];
        std::vector<float> Pomoc;
        Pomoc.push_back(novax);
        Pomoc.push_back(novay);

        dataVektor[0].push_back(Pomoc[0]);
        dataVektor[1].push_back(Pomoc[1]);
    }

    std::vector<float> distantniVektor;
    for(int i=0; i<dataVektor[0].size(); i++) {
        float vrijednost_distantnog_vektora=normVector[0]*dataVektor[0][i]+normVector[1]*dataVektor[1][i];//Upitan dio koda provjeri.
        distantniVektor.push_back(vrijednost_distantnog_vektora);

    }

    return distantniVektor;
}
//Glavna funkcija
std::vector<float> Ransac(std::vector<std::vector<float> > &ulazneTacke,std::vector<float> &Xkor,int k,float &zadnja_norma, int &broj_inlinera) {
// std::vector<float> vektorDistanceNormi;
    //ROS_INFO(" RADI na ulazu u Ransac \n");
    const int num=2;//Minimum number of points
    int iter=100;//ber of iterations(ASK DINKO)                }
    float treshold_distance=0.01;//DINKO                               }
    float inliner_ratio=0.2;//Upitni podaci,za ove parametre
    int number=ulazneTacke[0].size();                                         //}Ovi parametni treba da budu provjereni i stavljeni(eventualno) u deklaraciju funkcije
    std::vector<float> KiN;
    int bestInNum=0;
    float bestParameter1=0,bestParameter2=0;
    std::vector<int> vektorPozicijaZaBrisanje;
    int brojac=0;
    float n=0;
    for(int i=0; i<iter; i++) {
        std::vector<int> idx=randperm(number);
        std::vector<std::vector<float> > sample;
        sample.resize(ulazneTacke.size());
        int varijabla1=idx[0]-1;                  //Ispunjavamo sample sa nekakvim slucajnim tackama(Upitno je dali je ovaj kod ispravan)
        int varijabla2=idx[1]-1;
        sample[0].push_back(ulazneTacke[0][varijabla1]);
        sample[0].push_back(ulazneTacke[0][varijabla2]);
        sample[1].push_back(ulazneTacke[1][varijabla1]);//Vraća sample kkao treba,bez ikakvih problema
        sample[1].push_back(ulazneTacke[1][varijabla2]);
        float x_distance=sample[0][0]-sample[0][1];
        float y_distance=sample[1][0]-sample[1][1];

        float xnorm=x_distance/(sqrt(pow(x_distance,2)+pow(y_distance,2)));//Problem kod računanja ovih izraza
        float ynorm=y_distance/(sqrt(pow(x_distance,2)+pow(y_distance,2)));
        std::vector<float> normVector;
        normVector.push_back(-ynorm);
        normVector.push_back(xnorm);
        std::vector<float> distance=DistanceFunction(ulazneTacke,normVector,sample);
        std::vector<int> inlierIdx;
        float norma_distance_vektora=0;
        for(int i=0; i<distance.size(); i++) {
            float pomocna=abs(distance[i]);
            // ROS_INFO("POMOCNA: %.3f",pomocna);
            norma_distance_vektora+=distance[i]*distance[i];

            if(pomocna<=treshold_distance)inlierIdx.push_back(i);//Pretražuješ samo vektor
        }
        // ROS_INFO("Norma distance vektora: %.3f",norma_distance_vektora);
        norma_distance_vektora=sqrt(norma_distance_vektora);
        if(norma_distance_vektora!=norma_distance_vektora)norma_distance_vektora=(float)isnan(norma_distance_vektora);
        // ROS_INFO(" Korijenovana norma distance vektora: %.3f",norma_distance_vektora);
        n=norma_distance_vektora;
        int inlierNum=inlierIdx.size();
        float parameter1=0,parameter2=0;
        if(inlierNum>=round(inliner_ratio*number)&&inlierNum>bestInNum) {
            bestInNum=inlierNum;
            parameter1=(sample[1][1]-sample[1][0])/(sample[0][1]-sample[0][0]);
            parameter2=sample[1][0]-parameter1*sample[0][0]; //bestParameter1 i bestParameter2 daju tačke pravih
            bestParameter1=parameter1; //bestParameter1 i2 su maltene koeficijenti praca n i k prave y=nx+k;
            bestParameter2=parameter2;
            vektorPozicijaZaBrisanje=inlierIdx;
            // if(k==0){
            //  vektorDevijacija1.push_back(norma_distance_vektora);
            //  }else if(k==1){

            //   vektorDevijacija2.push_back(norma_distance_vektora);


            // }else{
            //   vektorDevijacija3.push_back(norma_distance_vektora);

            //  }
            //  brojac++;

        }


    }


    KiN.push_back(bestParameter1);
    KiN.push_back(bestParameter2);
    std::vector<float> Beskorisni;
    for(int j=0; j<vektorPozicijaZaBrisanje.size(); j++) {
        for(int i=0; i<ulazneTacke[0].size(); i++) {
            if(i==vektorPozicijaZaBrisanje[j]) {
                Beskorisni.push_back(ulazneTacke[0][i]);
                ulazneTacke[0].erase(ulazneTacke[0].begin()+i); //Kod za brisanje tacaka,inliner-a
                ulazneTacke[1].erase(ulazneTacke[1].begin()+i);
                for(int k=0; k<vektorPozicijaZaBrisanje.size(); k++)vektorPozicijaZaBrisanje[k]=vektorPozicijaZaBrisanje[k]-1;

            }


        }


    }
    zadnja_norma=n;
    broj_inlinera=bestInNum;
    float max_x=Max(Beskorisni);
    float min_x=Min(Beskorisni);
    Xkor.push_back(min_x);
    Xkor.push_back(max_x);
    //Ovaj ovdje dio što slijedi je čisto pokušaj,testiranje jednog moga mogućeg riješenja vezano za parametre Xmin i Xmax
    // if(k==0){
    //     vektorDevijacija1.push_back(max_x);
    //     vektorDevijacija4.push_back(min_x);
    //     }else if(k==1){

    //      vektorDevijacija2.push_back(max_x);
    //      vektorDevijacija5.push_back(min_x);

    //    }else{
    //      vektorDevijacija3.push_back(max_x);
    //      vektorDevijacija6.push_back(min_x);
    //     }
//float srednja_vrijednost=0;
//for(int i=0;i<vektorDistanceNormi.size();i++){

//   srednja_vrijednost+=vektorDistanceNormi[i];

//}
//float devijacije=0;
// for(int i=0;i<vektorDistanceNormi.size();i++){
    // float pomocna=vektorDistanceNormi[i]-srednja_vrijednost;
    // pomocna=pomocna*pomocna;
    // devijacije+=pomocna;


//}
//float sigma=devijacije/vektorDistanceNormi.size();
//sigma=sqrt(sigma);
//devijantni=sigma;
//ROS_INFO("Ovdje izadje iz Ransac-a\n");
    return KiN;

}
std::vector<float> minimalni(std::vector<std::vector<float> >A) {
    std::vector<float> pomoc1;
    std::vector<float> pomoc2;


    for(int i=0; i<A.size(); i++) {

        pomoc1.push_back(A[i][0]);
        pomoc2.push_back(A[i][1]);

    }
    std::vector<float> povratnik;
    float minimalna1=Min(pomoc1);
    float minimalna2=Min(pomoc2);
    povratnik.push_back(minimalna1);
    povratnik.push_back(minimalna2);
    return povratnik;
}
std::vector<float> maximalni(std::vector<std::vector<float> >A) {

    std::vector<float> pomoc1;
    std::vector<float> pomoc2;


    for(int i=0; i<A.size(); i++) {

        pomoc1.push_back(A[i][0]);
        pomoc2.push_back(A[i][1]);

    }
    std::vector<float> povratnik;
    float maximalna1=Max(pomoc1);
    float maximalna2=Max(pomoc2);
    povratnik.push_back(maximalna1);
    povratnik.push_back(maximalna2);
    return povratnik;
}



std::vector<std::vector<float> >Kmeans(std::vector<std::vector<float> >ulazneTacke,int nbCluster,std::vector<int> &pointsInCluster,std::vector<int> &assigment) {

    int data_dim=ulazneTacke[0].size();// je riječ o 2D tačkama
    int nbData=ulazneTacke.size();//Ukupan broj tacakaJer
    std::vector<float> data_min=minimalni(ulazneTacke);
    std::vector<float> data_max=maximalni(ulazneTacke);
    std::vector<float> data_diff;

    for(int i=0; i<data_min.size(); i++) {
        float razlika=data_max[i]-data_min[i];
        data_diff.push_back(razlika);


    }

    std::vector<std::vector<float> >Ones(nbCluster,std::vector<float>(data_dim,1));//Upitno,ako ne bud radilo,uradi standardnom procedurom
    std::vector<std::vector<float> >Rand;
    Rand.resize(nbCluster);
    for(int i=0; i<Rand.size(); i++)Rand[i].resize(data_dim);

    for(int i=0; i<Rand.size(); i++) {
        for(int j=0; j<Rand[i].size(); j++) {
            float varijabla=rand()/(float)(RAND_MAX);
            Rand[i][j]=varijabla;


        }

    }

    std::vector<std::vector<float> >Centroid;
    Centroid.resize(nbCluster);
    for(int i=0; i<Centroid.size(); i++)Centroid[i].resize(data_dim);

    for(int i=0; i<Centroid.size(); i++) {
        for(int j=0; j<Centroid[i].size(); j++) {
e slashes to specify the directo
            Centroid[i][j]=Ones[i][j]*Rand[i][j];

        }
    }

    for(int i=0; i<Centroid.size(); i++) {
        for(int j=0; j<Centroid[i].size(); j++) {
            Centroid[i][j]=Centroid[i][j]*data_diff[j];
            Centroid[i][j]=Centroid[i][j]+data_min[j];
        }
    }

    float pos_dif=1.0;
    while(pos_dif>0.0) {
        std::vector<int> asignment;
        for(int i=0; i<ulazneTacke.size(); i++) {
            std::vector<float> min_diff;
            for(int j=0; j<Centroid[0].size(); j++) {
                float pomocna=0.0;
                pomocna=ulazneTacke[i][j]-Centroid[0][j];
                min_diff.push_back(pomocna);


            }
            float min_dif=0;
            for(int j=0; j<min_diff.size(); j++) {
                min_dif+=min_diff[j]*min_diff[j];


            }
            int curAssignmetn=0;
            for(int k=1; k<nbCluster; k++) {
                std::vector<float> diff2c;
                float pomocna=0;
                for(int j=0; j<Centroid[k].size(); j++) {
                    float pomocna=ulazneTacke[i][j]-Centroid[k][j];
                    diff2c.push_back(pomocna);
                }
                float dif2c=0.0;
                for(int j=0; j<diff2c.size(); j++) {
                    dif2c+=diff2c[j]*diff2c[j];

                }
                if(min_dif>=dif2c) {

                    curAssignmetn=k;
                    min_dif=dif2c;

                }



            }
            asignment.push_back(curAssignmetn);
        }

        std::vector<std::vector<float> > oldPositions=Centroid;
        for(int i=0; i<Centroid.size(); i++) {
            for(int j=0; j<Centroid[i].size(); j++) {

                Centroid[i][j]=0.0;

            }


        }
        std::vector<int> pomocniPointsInCluster;

        for(int i=0; i<nbCluster; i++) {
            pomocniPointsInCluster.push_back(0);

        }
        for(int i=0; i<asignment.size(); i++) {
            int pomocna=asignment[i];
            for(int j=0; j<Centroid[pomocna].size(); j++) {
                Centroid[pomocna][j]+=ulazneTacke[i][j];

            }

            pomocniPointsInCluster[pomocna]=pomocniPointsInCluster[pomocna]+1;





        }

        for(int i=0; i<nbCluster; i++) {

            if(pomocniPointsInCluster[i]!=0) {

                for(int j=0; j<Centroid[i].size(); j++) {

                    Centroid[i][j]=Centroid[i][j]/(float)pomocniPointsInCluster[i];

                }




            }
            else {
                std::vector<float> randoza;
                randoza.resize(data_dim);
                for(int k=0; k<randoza.size(); k++) {

                    float varijabla=rand()/(float)(RAND_MAX);
                    randoza[k]=varijabla;
                }

                for(int j=0; j<Centroid[i].size(); j++) {


                    Centroid[i][j]=randoza[j]*data_diff[j]+data_min[j];


                }





            }




        }

        std::vector<std::vector<float> >pomocni;
        pomocni.resize(nbCluster);
        for(int i=0; i<pomocni.size(); i++)pomocni[i].resize(data_dim);


        for(int i=0; i<pomocni.size(); i++) {
            for(int j=0; j<pomocni[i].size(); j++) {

                pomocni[i][j]=Centroid[i][j]-oldPositions[i][j];
                pomocni[i][j]=pomocni[i][j]*pomocni[i][j];
            }


        }
        std::vector<float> vektorsuma;
        for(int i=0; i<pomocni.size(); i++) {
            float pomocna=0;
            for(int j=0; j<pomocni[i].size(); j++) {

                pomocna+=pomocni[i][j];


            }
            vektorsuma.push_back(pomocna);


        }
        float povratna=0.0;
        for(int i=0; i<vektorsuma.size(); i++) {
            povratna+=vektorsuma[i];
        }
        pos_dif=povratna;
        assigment=asignment;
        pointsInCluster=pomocniPointsInCluster;
    }

//Ovaj dio koda dole sto je napisan odnosi se na pokusaj implementacije da se vrati i odogovaraći radijus vektori
    std::vector<std::vector<float> >noviVektorTacaka;
    noviVektorTacaka.resize(ulazneTacke.size());


    for(int i=0; i<noviVektorTacaka.size(); i++) {
        noviVektorTacaka[i].push_back(ulazneTacke[i][0]);
        noviVektorTacaka[i].push_back(ulazneTacke[i][1]);
        noviVektorTacaka[i].push_back(assigment[i]);



    }
    std::vector<std::vector<float> >noviVektorCentroida;
    noviVektorCentroida.resize(Centroid.size());
    for(int i=0; i<noviVektorCentroida.size(); i++) {
        noviVektorCentroida[i].push_back(Centroid[i][0]);
        noviVektorCentroida[i].push_back(Centroid[i][1]);
        noviVektorCentroida[i].push_back(i);



    }
    std::vector<std::vector<float> >radijusi;
    radijusi.resize(Centroid.size());
    for(int i=0; i<nbCluster; i++) {
        std::vector<float> pomocniVektorZaProracun;
        for(int j=0; j<noviVektorTacaka.size(); j++) {


            if(i==noviVektorTacaka[j][2]) {

                for(int k=0; k<noviVektorCentroida.size(); k++) {

                    if(i==noviVektorCentroida[k][2]) {
                        float metrika=0;
                        float udaljenost_x=noviVektorCentroida[k][0]-noviVektorTacaka[j][0];
                        float udaljenost_y=noviVektorCentroida[k][1]-noviVektorTacaka[j][1];
                        float kvadrirana_udaljenost=udaljenost_x*udaljenost_x+udaljenost_y*udaljenost_y;
                        metrika=sqrt(kvadrirana_udaljenost);
                        pomocniVektorZaProracun.push_back(metrika);



                    }




                }






            }






        }
        float povratni_radijus=0;
        povratni_radijus=Max(pomocniVektorZaProracun);
        if(povratni_radijus>0.5)povratni_radijus=0;
        radijusi[i].push_back(povratni_radijus);
        radijusi[i].push_back(i);







    }






    std::vector<std::vector<float> >CentoridSaRadijusom;
    CentoridSaRadijusom.resize(Centroid.size());
    for(int i=0; i<CentoridSaRadijusom.size(); i++) {

        CentoridSaRadijusom[i].push_back(Centroid[i][0]);
        CentoridSaRadijusom[i].push_back(Centroid[i][1]);
        CentoridSaRadijusom[i].push_back(radijusi[i][0]);


    }
   // ROS_INFO("Izaslo \n");
    return CentoridSaRadijusom;

}

std::vector<std::vector<float> >Pretvaranje(const sensor_msgs::LaserScan &scan) {
    std::vector<std::vector<float> >PovratniVektor;
//Dio koda koji slijedi mora se provjeriti s DInkom,rađen je na osnovu predloška sa ROS.org answersa.
    PovratniVektor.resize(2);
    //Pokusaj implementiranja poopćenog djela f-je 
    float angular_resolution=(scan.angle_max-scan.angle_min)/scan.ranges.size();
    float pola_ugla=(scan.angle_max-scan.angle_min)/2;
    for(int i=0; i<PovratniVektor.size(); i++)PovratniVektor[i].resize(360);
    for(int i=0; i<scan.ranges.size(); i++) {
      float phi=(i*angular_resolution)-pola_ugla;
      //  float phi_degrees=(i * 0.5)-90;
      //  float phi=(phi_degrees*3.14)/180;
        float x=scan.ranges[i]*cos(phi);
        float y=scan.ranges[i]*sin(phi);
        PovratniVektor[0][i]=x;
        PovratniVektor[1][i]=y;
//ROS_INFO("%.3f %.3f %.3f\n", x, y, scan.ranges[i]);


    }

    return PovratniVektor;
}

//float standardna_devijacija(std::vector<float> vektorDevijacija){
//  std::vector<float> pomocni;
//  if(vektorDevijacija.size()>10){

//  for(int i=vektorDevijacija.size()-10;i<vektorDevijacija.size()-1;i++){
// pomocni.push_back(vektorDevijacija[i]);
// }

//} else{

//   for(int i=0;i<vektorDevijacija.size();i++){

//     pomocni.push_back(vektorDevijacija[i]);



//   }




//  }


// float srednja_vrijednost=0;
//  for(int i=0;i<pomocni.size();i++){
//      srednja_vrijednost+=pomocni[i];




//  }
// srednja_vrijednost=srednja_vrijednost/pomocni.size();
//   float varijansa=0;
// for(int i=0;i<pomocni.size();i++){
//    float pomocna=0;
//    pomocna=pomocni[i]-srednja_vrijednost;
//    pomocna=pow(pomocna,2);
//    varijansa+=pomocna;



//   }
// varijansa=varijansa/pomocni.size();
// float sigma=0;
//  sigma=sqrt(varijansa);
// ROS_INFO("Izadje iz f-je devijacija\n");//Ovo je sada dodatni dio,kad uzmeš rekurzivnu varijaciju


// return sigma;
//return varijansa;
//Ovaj dole dio je pokušaj implementacije Dinkove ideje

//}
//float s1_t=0;
//float mu1_t=0;
//float x1_t=0;
//int t=1;
//float s2_t=0;
//float mu2_t=0;
//float x2_t=0;
//float s3_t=0;
//float mu3_t=0;
//float x3_t=0;
ros::Publisher marker_pub;
//std::vector<float>vektorDevijacija;
void Algoritam(const sensor_msgs::LaserScan &scan) {
//Ovo je kao neki glavni program maltene,ovdje pokrecemo algoritme trpamo u vektore i koristimo ROS markere te pozivamo program za petvaranje tacaka i vizualizaciju
    std::vector<std::vector<float> >Podaci=Pretvaranje(scan);
    std::vector<float> Xkor;
    std::vector<std::vector<float> >VektorKINova;
    VektorKINova.resize(3);
    std::vector<std::vector<float> >VektorXminIXmax;
    VektorXminIXmax.resize(3);
   // myfile.open("pravac1-slučaj7.txt",ios::app);
   // drugifile.open("pravac2-slučaj7.txt",ios::app);
   // trecifile.open("pravac3-slučaj7.txt", ios::app);
    for(int i=0; i<3; i++) {
// float devijantni=0;
//float devijantni1;
        float zadnja_norma=0;
        int inliner_num;
        std::vector<float> pomocRansac;
        std::vector<float> pomocXkor;
        pomocRansac=Ransac(Podaci,pomocXkor,i,zadnja_norma,inliner_num);

        if(i==0) {
//devijantni=standardna_devijacija(vektorDevijacija1);
//devijantni1=standardna_devijacija(vektorDevijacija4);
// float pomocna=zadnja_norma;
// float mu_t1=(float)t/(t+1)*mu1_t+1.0/(t+1)*pomocna;
// float s_t1=(float)t/(t+1)*s1_t+1.0/t*(pomocna-mu_t1)*(pomocna-mu_t1);
// x1_t=pomocna;
// s1_t=s_t1;
// mu1_t=mu_t1;
           // myfile<<zadnja_norma<<"       "<<inliner_num<<"\n";
        }
        else if(i==1) {
// devijantni=standardna_devijacija(vektorDevijacija2);
            //devijantni1=standardna_devijacija(vektorDevijacija5);
// float pomocna=zadnja_norma;
// myfile<<pomocna<<"\n";
// float mu_t1=(float)t/(t+1)*mu2_t+1.0/(t+1)*pomocna;
// float s_t1=(float)t/(t+1)*s2_t+1.0/t*(pomocna-mu_t1)*(pomocna-mu_t1);
// x2_t=pomocna;
// s2_t=s_t1;
// mu2_t=mu_t1;
           // drugifile<<zadnja_norma<<"          "<<inliner_num<<"\n";
        } else {
// devijantni=standardna_devijacija(vektorDevijacija3);
            //devijantni1=standardna_devijacija(vektorDevijacija6);
//  float pomocna=zadnja_norma;
//  drugifile<<pomocna<<"\n";
//  float mu_t1=(float)t/(t+1)*mu3_t+1.0/(t+1)*pomocna;
//  float s_t1=(float)t/(t+1)*s3_t+1.0/t*(pomocna-mu_t1)*(pomocna-mu_t1);
//  x3_t=pomocna;
//  s3_t=s_t1;
//  mu3_t=mu_t1;
          //  trecifile<<zadnja_norma<<"             "<<inliner_num<<"\n";
        }
// myfile<<devijantni<<"\n";
// ROS_INFO("Devijantni:%.3f\n",devijantni);
// if(i==1){
//  if(s2_t>0.1){
//   pomocRansac[0]=0;
//   pomocRansac[1]=0;
//   pomocXkor[0]=999999;
//   pomocXkor[1]=999999;

// }
//}//else if(i==1){

        // if(s2_t>150.00){
        //  pomocRansac[0]=0;
        //  pomocRansac[1]=0;
        //  pomocXkor[0]=999999;
        //  pomocXkor[1]=999999;

        //   }

//}
//   else if(i==2){

//   if(s3_t>0.001){
//    pomocRansac[0]=0;
//    pomocRansac[1]=0;
//    pomocXkor[0]=999999;
//    pomocXkor[1]=999999;
        //
//  }
        VektorKINova[i].push_back(pomocRansac[0]);
        VektorKINova[i].push_back(pomocRansac[1]);
        VektorXminIXmax[i].push_back(pomocXkor[0]);
        VektorXminIXmax[i].push_back(pomocXkor[1]);
    }

    visualization_msgs::MarkerArray prepreke;
    for(int i=0; i<VektorKINova.size(); i++) {

        // ROS_INFO(" k=%f n=%f",VektorKINova[i][0],VektorKINova[i][1]);
        // ROS_INFO(" Xmin=%f,Xmax=%f",VektorXminIXmax[i][0],VektorXminIXmax[i][1]);




    }



//Ovdje ide sada blok koda koji omogućava Vizualizaciju 3 pravca+Kmeans
    visualization_msgs::Marker tacke,line_strip1,line_strip2,line_strip3;
    tacke.header.frame_id=line_strip1.header.frame_id=line_strip2.header.frame_id=line_strip3.header.frame_id="/my_frame";
    tacke.header.stamp=line_strip1.header.stamp=line_strip2.header.stamp=line_strip3.header.stamp=ros::Time::now();
    tacke.ns=line_strip1.ns=line_strip2.ns=line_strip3.ns="detection";
    tacke.pose.orientation.w=line_strip1.pose.orientation.w=line_strip2.pose.orientation.w=line_strip3.pose.orientation.w=1.0;
    tacke.action=line_strip1.action=line_strip2.action=line_strip3.action=visualization_msgs::Marker::ADD;
    tacke.type=visualization_msgs::Marker::POINTS;
    line_strip1.type=line_strip2.type=line_strip3.type=visualization_msgs::Marker::LINE_STRIP;
    tacke.id=0;
    line_strip1.id=1;
    line_strip2.id=2;
    line_strip3.id=3;
    tacke.color.g=1.0;
    tacke.color.a=1.0;
    line_strip1.color.b=line_strip2.color.b=line_strip3.color.b=1.0;
    line_strip1.color.a=line_strip2.color.a=line_strip3.color.a=1.0;

    tacke.scale.x=0.2;
    tacke.scale.y=0.2;
    line_strip1.scale.x=line_strip2.scale.x=line_strip3.scale.x=0.1;
    geometry_msgs::Point p1,p2,p3,p4,p5,p6;
    float y1,y2,y3,y4,y5,y6;
    y1=VektorKINova[0][0]*VektorXminIXmax[0][0]+VektorKINova[0][1];
    p1.x=VektorXminIXmax[0][0];
    p1.y=y1;
    p1.z=0;
    y2=VektorKINova[0][0]*VektorXminIXmax[0][1]+VektorKINova[0][1];
    p2.x=VektorXminIXmax[0][1];
    p2.y=y2;
    p2.z=0;
    y3=VektorKINova[1][0]*VektorXminIXmax[1][0]+VektorKINova[1][1];
    p3.x=VektorXminIXmax[1][0];
    p3.y=y3;
    p3.z=0;
    y4=VektorKINova[1][0]*VektorXminIXmax[1][1]+VektorKINova[1][1];
    p4.x=VektorXminIXmax[1][1];
    p4.y=y4;
    p4.z=0;

    y5=VektorKINova[2][0]*VektorXminIXmax[2][0]+VektorKINova[2][1];
    p5.x=VektorXminIXmax[2][0];
    p5.y=y5;
    p5.z=0;

    y6=VektorKINova[2][0]*VektorXminIXmax[2][1]+VektorKINova[2][1];
    p6.x=VektorXminIXmax[2][1];
    p6.y=y6;
    p6.z=0;



//ROS_INFO("TACKE p1 i p2: (%.3f %.3f) \t (%.3f %.3f) \n", p1.x, p1.y, p2.x, p2.y);
//  ROS_INFO("TACKE p3 i p4: (%.3f %.3f) \t (%.3f %.3f) \n", p3.x, p3.y, p4.x, p4.y);
//  ROS_INFO("TACKE p5 i p6: (%.3f %.3f) \t (%.3f %.3f) \n", p5.x, p5.y, p6.x, p6.y);



    tacke.points.push_back(p1);
    tacke.points.push_back(p2);
    tacke.points.push_back(p3);
    tacke.points.push_back(p4);
    tacke.points.push_back(p5);
    tacke.points.push_back(p6);
    line_strip1.points.push_back(p1);
    line_strip1.points.push_back(p2);
    line_strip2.points.push_back(p3);
    line_strip2.points.push_back(p4);
    line_strip3.points.push_back(p5);
    line_strip3.points.push_back(p6);
    prepreke.markers.push_back(tacke);
    prepreke.markers.push_back(line_strip1);
    prepreke.markers.push_back(line_strip2);
    prepreke.markers.push_back(line_strip3);
    std::vector<int> pointsInCluster;
    std::vector<int> assignment;
 //   ROS_INFO("Ovdje dodjes \n");
    std::vector<std::vector<float> >ModifikovaneTacke;
    ModifikovaneTacke.resize(Podaci[0].size());
    for(int i=0; i<ModifikovaneTacke.size(); i++) {
        ModifikovaneTacke[i].push_back(Podaci[0][i]);
        ModifikovaneTacke[i].push_back(Podaci[1][i]);


    }

    //ROS_INFO("I ovdje dodjes,Modi:%d \n",broj);
    int nbCluster=5;//Ovo primjeti,moraš jer Kmeans je takav algoritam da traži unaprijed definiran broj clustera
    if(ModifikovaneTacke.size()>=5){
    std::vector<std::vector<float> >Kruznice=Kmeans(ModifikovaneTacke,nbCluster,pointsInCluster,assignment);
    // ROS_INFO("Ovdje isto \n");
    for(int i=0;i<nbCluster;i++){
    visualization_msgs::Marker pomoc;

       pomoc.header.frame_id="/my_frame";
       pomoc.header.stamp=ros::Time::now();
       pomoc.ns="detection";
       pomoc.pose.orientation.w=1.0;
       pomoc.action=visualization_msgs::Marker::ADD;
       pomoc.type=visualization_msgs::Marker::CYLINDER;
       pomoc.id=i+4;
       pomoc.color.r=1.0;
       pomoc.color.a=1.0;

       pomoc.scale.x=Kruznice[i][2];
       pomoc.scale.y=Kruznice[i][2];
       pomoc.scale.z=0;

       pomoc.pose.position.x=Kruznice[i][0];
       pomoc.pose.position.y=Kruznice[i][1];
       pomoc.pose.position.z=0;

       prepreke.markers.push_back(pomoc);






       }
   }

    marker_pub.publish(prepreke);
   // myfile.close();
   // drugifile.close();
   // trecifile.close();

// t++;







}
int main(int argc,char **argv) {
//Ovaj main dio se koristi za pretplacivanje na temu koja nam daje laserska mjerenja

    ros::init(argc,argv,"laser");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("base_scan",10,Algoritam);
    ros::NodeHandle r;
    ros::Rate loop_rate(1);
    marker_pub=r.advertise<visualization_msgs::MarkerArray>("markers",10);
    ros::spin();
    return 0;
}
