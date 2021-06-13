import numpy as np

refs_x=[1000,1000,0,0]
refs_y=[0,1000,1000,0]
def recortar_referencias(ref_x,ref_y):
    #numero de referencias
    largo=len(ref_x)
    #vectores finales
    ref_x_cortada=[]
    ref_y_cortada=[]
    for i in range(largo):
        #calcular norma, angulo y posicion anterior
        if i==0:
            raiz_norma=ref_x[0]**2+ref_y[0]**2
            pos_anterior=[0,0]
            angulo=np.arctan2(ref_y[0], ref_x[0])
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
            if next_x<1:
                ref_x_cortada.append(0)
            else:
                ref_x_cortada.append(pos_anterior[0]+np.cos(angulo)*200*(j+1))
            if next_y<1:
                ref_y_cortada.append(0)
            else:
                ref_y_cortada.append(pos_anterior[1]+np.sin(angulo)*200*(j+1))
        if (mag-numero_de_puntos*200)>100:
            ref_x_cortada.append(pos_anterior[0]+np.cos(angulo)*200*(numero_de_puntos*20+mag_int%20))
            ref_y_cortada.append(pos_anterior[1]+np.sin(angulo)*200*(numero_de_puntos*20+mag_int%20))
    return [ref_x_cortada,ref_y_cortada]

print(recortar_referencias(refs_x,refs_y))






        