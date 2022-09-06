import numpy as np
from descomponer import recortar_referencias

##todo en milímetros
def nueva_ref(referencias,index_ref,pos_x,pos_y,theta):
    refs_x=[]
    refs_y=[]
    for j in range(len(referencias)):
        refs_x.append(referencias[j][0])
        refs_y.append(referencias[j][1])

    print(refs_x)
    print(refs_y)
    dist_a_recorrer=400.0
    ##rotar noventa grados
    theta_aux = theta + (3.14 / 2.0)
    ##nueva referencia
    ref_x_aux = pos_x + np.cos(theta_aux)*dist_a_recorrer
    ref_y_aux = pos_y + np.sin(theta_aux)*dist_a_recorrer

    #eliminar las próximas n referencias
    n=3
    if (index_ref + n +1)<(len(refs_y)):
        lista_intermedia=[]
        lista_intermedia=recortar_referencias([[refs_x[(index_ref+n)],refs_y[(index_ref+n)]]],int(ref_x_aux),int(ref_y_aux))
        refs_x_aux=[ref_x_aux]
        refs_y_aux=[ref_y_aux]
        for j in range(len(lista_intermedia)):
            refs_x_aux.append(lista_intermedia[j][0])
            refs_y_aux.append(lista_intermedia[j][1])


        #colocar nueva referencia
        #refs_x.insert((index_ref), ref_x_aux)
        #refs_y.insert((index_ref), ref_y_aux)



        #generar copias
        refs_x_aux_1=refs_x.copy()
        refs_y_aux_1=refs_y.copy()
        ##cortar
        ##falta revisar casos borde
        refs_x_aux_1=refs_x_aux_1[0:(index_ref )]
        refs_x_aux_2=refs_x[(index_ref + n +1 ):]
        refs_y_aux_1=refs_y_aux_1[0:(index_ref )]
        refs_y_aux_2=refs_y[(index_ref + n +1 ):]
        ##unir listas
        refs_x_aux_1.extend(refs_x_aux)
        refs_y_aux_1.extend(refs_y_aux)
        refs_x_aux_1.extend(refs_x_aux_2)
        refs_y_aux_1.extend(refs_y_aux_2)
    else:
        #generar copias
        refs_x_aux_1=refs_x.copy()
        refs_y_aux_1=refs_y.copy()
        j=1
        while j<=n:
            j_aux=j*(-1)
            refs_x_aux_1[j_aux]=pos_x
            refs_y_aux_1[j_aux]=pos_y
            j+=1
        
    lista_final=[]      
    ##print(refs_x_aux_1)  
    ##print(refs_y_aux_1)
    for j in range(len(refs_x_aux_1)):
        lista_final.append([int(refs_x_aux_1[j]),int(refs_y_aux_1[j])])


    #retornar listas
    return lista_final

refs=[[200, 0], [400, 0], [600, 0], [800, 0], [1000, 0], [1000, 200], [1000, 400], [1000, 600], [1000, 800], [1000, 1000], [800, 1000], [600, 1000], [400, 1000], [200, 1000], [0, 1000], [0, 800], [0, 600], [0, 400], [0, 200], [0, 0]]
index_ref=1
posx=300
posy=0.0
theta=0.0

nueva_ref(refs,index_ref,posx,posy,theta)
