import numpy as np
from descomponer import recortar_referencias

##todo en milímetros
def nueva_ref(pos_x,pos_y,theta):
    dist_a_recorrer=2000.0
    ##rotar noventa grados
    theta_aux = theta + (3.14 / 2.0)
    ##nueva referencia
    ref_x_aux = pos_x + np.cos(theta_aux)*dist_a_recorrer
    ref_y_aux = pos_y + np.sin(theta_aux)*dist_a_recorrer
    lista_final = []
    lista_final=recortar_referencias([[ref_x_aux,ref_y_aux]],pos_x,pos_y)


    #retornar listas
    return lista_final

