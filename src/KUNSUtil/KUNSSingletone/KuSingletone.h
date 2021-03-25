/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : 싱글톤 타입으로 클래스를 생성해주는 기능을 제공하는 클래스.
$Created on: 2012. 5. 4.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef KUNS_SINGLETONE_H_
#define KUNS_SINGLETONE_H_

#include <iostream>
#include <memory>

using namespace std;

template<typename T>
class KuSingletone
{
private:
	static auto_ptr<T> thisInstance;


public:
	 KuSingletone(){}
     virtual ~KuSingletone(){}

     static T* getInstance(){
       	 if(NULL == thisInstance.get()){
       	 	auto_ptr<T> pTemp(new T);
       	 	thisInstance = pTemp;
       	 }
       	 return thisInstance.get();
        }

};

template <typename T> auto_ptr<T> KuSingletone<T>::thisInstance;;    //definition and initialization of singletone instance

#endif /* KUNS_SINGLETONE_H_ */

