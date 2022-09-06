import numpy as np



def recortar_referencias(refs,pos_x,pos_y):
    ref_x=[]
    ref_y=[]
    for j in range(len(refs)):
        ref_x.append(refs[j][0])
        ref_y.append(refs[j][1])

    #numero de referencias
    largo=len(ref_x)
    #vectores finales
    ref_x_cortada=[]
    ref_y_cortada=[]
    for i in range(largo):
        #calcular norma, angulo y posicion anterior
        if i==0:
            raiz_norma=(ref_x[0]-pos_x)**2+(ref_y[0]-pos_y)**2
            pos_anterior=[pos_x,pos_y]
            angulo=np.arctan2((ref_y[0]-pos_y), (ref_x[0]-pos_x))
        else:
            raiz_norma=(ref_x[i]-ref_x[i-1])**2+(ref_y[i]-ref_y[i-1])**2
            pos_anterior=[ref_x[i-1],ref_y[i-1]]
            angulo=np.arctan2(ref_y[i]-ref_y[i-1], ref_x[i]-ref_x[i-1])
        mag=np.sqrt(raiz_norma)
        mag_int=int(mag)
        numero_de_puntos=mag_int//200
        for j in range(numero_de_puntos):
            next_x=pos_anterior[0]+np.cos(angulo)*200*(j+1)
            next_y=pos_anterior[1]+np.sin(angulo)*200*(j+1)
            if (next_x<1) and ((-1)<next_x):
                ref_x_cortada.append(0)
            else:
                ref_x_cortada.append(pos_anterior[0]+np.cos(angulo)*200*(j+1))
            if (next_y<1) and ((-1)<next_y):
                ref_y_cortada.append(0)
            else:
                ref_y_cortada.append(pos_anterior[1]+np.sin(angulo)*200*(j+1))
        if (mag-numero_de_puntos*200)>100:
            ref_x_cortada.append(pos_anterior[0]+np.cos(angulo)*(200*(numero_de_puntos)+(mag-numero_de_puntos*200)))
            ref_y_cortada.append(pos_anterior[1]+np.sin(angulo)*(200*(numero_de_puntos)+(mag-numero_de_puntos*200)))
            
    lista_final=[]        
    for j in range(len(ref_x_cortada)):
        lista_final.append([int(ref_x_cortada[j]),int(ref_y_cortada[j])])
    
    return lista_final


#refs=recortar_referencias([[600,0],[600,-600],[0,-600]],0,0)

#ref_x=[]
#ref_y=[]
#for j in range(len(refs)):
#    ref_x.append(refs[j][0])
#    ref_y.append(refs[j][1])

#print(ref_x)
#print(ref_y)