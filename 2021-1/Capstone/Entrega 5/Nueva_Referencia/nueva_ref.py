import numpy as np


##todo en milímetros
def nueva_ref(referencias,index_ref,pos_x,pos_y,theta,distancias):
    refs_x=[]
    refs_y=[]
    for j in range(len(referencias)):
        refs_x.append(referencias[j][0])
        refs_y.append(referencias[j][1])

    ##distancia y ángulo al punto más cercano
    l_1=float(distancias[0][0])*10.0
    q_1=float(distancias[0][1])/180.0*np.pi
    ##distancia y ángulo al extremo más cercano
    l_2=float(distancias[1][0])*10.0
    q_2=float(distancias[1][1])/180.0*np.pi

    ##posicion punto más cercano
    x1= pos_x + np.cos(theta + q_1)*l_1
    y1= pos_y + np.sin(theta + q_1)*l_1
    ##posicion extremo más cercano
    x2= pos_x + np.cos(theta + q_2)*l_2
    y2= pos_y + np.sin(theta + q_2)*l_2

    #norma vector p2-p1
    norma=np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    ##nueva referencia
    x3=x2 + (x2 - x1)/norma*200.0
    y3=y2 + (y2 - y1)/norma*200.0
    #colocar nueva referencia
    refs_x.insert((index_ref - 1), x3)
    refs_y.insert((index_ref - 1), y3)

    #eliminar las próximas n referencias
    n=1
    #generar copias
    refs_x_aux_1=refs_x.copy()
    refs_y_aux_1=refs_y.copy()
    ##cortar
    ##falta revisar casos borde
    refs_x_aux_1=refs_x_aux_1[0:(index_ref )]
    refs_x_aux_2=refs_x[(index_ref + n ):]
    refs_y_aux_1=refs_y_aux_1[0:(index_ref )]
    refs_y_aux_2=refs_y[(index_ref + n ):]
    ##unir listas
    refs_x_aux_1.extend(refs_x_aux_2)
    refs_y_aux_1.extend(refs_y_aux_2)
    
    lista_final=[]        
    for j in range(len(refs_x_aux_1)):
        lista_final.append([int(refs_x_aux_1[j]),int(refs_y_aux_1[j])])


    #retornar listas
    return lista_final

refs=[[200, 0], [400, 0], [600, 0], [800, 0], [1000, 0], [1000, 200], [1000, 400], [1000, 600], [1000, 800], [1000, 1000], [800, 1000], [600, 1000], [400, 1000], [200, 1000], [0, 1000], [0, 800], [0, 600], [0, 400], [0, 200], [0, 0]]
index_ref=3
posx=400.0
posy=0.0
theta=0.0
dists=[[14.1,0.0],[17.286,35.34]]

print(nueva_ref(refs,index_ref,posx,posy,theta,dists))
