#!/usr/bin/env python
# coding: utf-8

# In[58]:


class VectorCartesiano: #Definimos la clase
    magnitud = 0
    def __init__(self,x0=0,y0=0,z0=0):
        self.x=x0
        self.y=y0
        self.z=z0
        self.cartesiano=[x0,y0,z0]
        self.magnitudCartesiano=(x0**2 + y0**2 + z0**2)**0.5
    def __mul__(self,other): #Sobrecargamos el método multiplicación
        return self.x*other.x + self.y*other.y + self.z*other.z
    def Cruz(self,other): #Definimos el producto cruz
        return VectorCartesiano(self.y*other.z - self.z*other.y,-self.x*other.z + self.z*other.x,self.x*other.y - self.y*other.x)
    
    def __add__(self,other): #Sobrecargamos el método suma
        return VectorCartesiano(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self,other): #Sobrecargamos el método resta
        return VectorCartesiano(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __getitem__(self,index):
        return self.cartesiano[index]
    def __eq__(self,other):
        return (self.x,self.y,self.z)==(other.x,other.y,other.z)
    def Print(self):
        print(f"[{self.x},{self.y},{self.z}]")
    #def __eq__()
    
    def TransPolar(self): #Transformación de los atributos cartesianos a esfericas
        import math as m
        
        #Solucionamos el problema de la tangente inversa, dado que nada más va de -pi/2 a pi/2
        if self.x==0:
            if abs(self.y)==self.y:
                h=VectorCartesiano(self.magnitudCartesiano,m.acos(self.z/self.magnitudCartesiano),m.pi/2)
            else:
                h=VectorCartesiano(self.magnitudCartesiano,m.acos(self.z/self.magnitudCartesiano),-m.pi/2)
    
        elif self.x>0 and self.y>=0:
            h=VectorCartesiano(self.magnitudCartesiano,m.acos(self.z/self.magnitudCartesiano),m.tan(self.y/self.x))
        elif self.x>0 and self.y<=0:
            h=VectorCartesiano(self.magnitudCartesiano,m.acos(self.z/self.magnitudCartesiano),2*m.pi+m.atan(self.y/self.x))
        elif self.x<0 :
            h=VectorCartesiano(self.magnitudCartesiano,m.acos(self.z/self.magnitudCartesiano),m.pi+m.atan(self.y/self.x))
        return h
        


# In[133]:


class VectorPolar(VectorCartesiano):
    import numpy as np
    
    def __init__(self,r,theta,phi):
        import numpy as np
        self.r=abs(r) #r no puede tomar valores negativos
        self.theta=theta
        self.phi=phi
        #Vamos a pasar a reescalar tanto theta como phi, que barran valores entre 0 y pi, y 0 y 2pi
        #respectivamente
        t = theta
        k= int(t/np.pi)
        angulot = t-k*np.pi #ángulo reescalado
                 #Tomamos los dos posibles casos de que sea positivo o negativo
        if t==0:
            theta = 0
        elif t>0:
            if angulot==0:
                theta = np.pi
            else:
                theta=angulot
        self.theta=theta
        #Reescalamos phi
        
        p = phi
        n = int(p/(2*np.pi))
        angulop = p- 2*n*np.pi
        if p>0:
            phi=angulop
        else:
            phi= angulop + 2*np.pi
        self.phi=phi
        
        self.polar = [self.r,self.theta,self.phi]
        self.magnitudPolar = self.r
        VectorCartesiano.__init__(self,self.r*np.sin(self.theta)*np.cos(self.phi),
                                  self.r*np.sin(self.theta)*np.sin(self.phi),
                                  self.r*np.cos(self.theta)) # Transformación de las componentes en cartesianas
        
        
    def TransCar(self): #En cartesianas
        import numpy as np
        return VectorCartesiano(self.r*np.sin(self.theta)*np.cos(self.phi),
                                  self.r*np.sin(self.theta)*np.sin(self.phi),
                                  self.r*np.cos(self.theta))
        
    def TransEsf(self):
        import numpy as np
        return VectorCartesiano(self.r*np.sin(self.theta)*np.cos(self.phi)*np.sin(self.theta)*np.cos(self.phi) +
                self.r*np.sin(self.theta)*np.sin(self.phi)*np.sin(self.theta)*np.sin(self.phi) +
                self.r*np.cos(self.theta)*np.cos(self.theta),self.r*np.sin(self.theta)*np.cos(self.phi)*
                np.cos(self.theta)*np.cos(self.phi) + self.r*np.sin(self.theta)*np.sin(self.phi)*
                np.cos(self.theta)*np.sin(self.phi) - self.r*np.cos(self.theta)*np.sin(self.theta),
                -self.r*np.sin(self.theta)*np.cos(self.phi)*np.sin(self.phi) + 
                self.r*np.sin(self.theta)*np.sin(self.phi)*np.cos(self.phi)) #Transformación de los elementos de
                                                                                # la base
                
   #Con esta transformació ya hemos reparado TODO     
   
        

                                         
                                         

