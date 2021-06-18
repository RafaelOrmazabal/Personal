import numpy as np


##todo en milímetros
def nueva_ref(referencias,index_ref,pos_x,pos_y,theta,distancias):
    refs_x=referencias[0]
    refs_y=referencias[1]

    ##distancia y ángulo al punto más cercano
    l_1=float(distancias[0][0])*10.0
    q_1=float(distancias[0][1])/180.0*np.pi
    ##distancia y ángulo al extremo más cercano
    l_2=float(distancias[1][0])*10.0
    q_2=float(distancias[1][1])/180.0*np.pi

    ##posicion punto más cercano
    x1= pos_x + np.cos(theta - q_1)*l_1
    y1= pos_y + np.sin(theta - q_1)*l_1
    ##posicion extremo más cercano
    x2= pos_x + np.cos(theta - q_2)*l_2
    y2= pos_y + np.sin(theta - q_2)*l_2

    #norma vector p2-p1
    norma=np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    ##nueva referencia
    x3=x2 + (x2 - x1)/norma*200.0
    y3=y2 + (y2 - y1)/norma*200.0
    #colocar nueva referencia
    refs_x.insert((index_ref-1), x3)
    refs_y.insert((index_ref-1), y3)

    return [refs_x,refs_y]