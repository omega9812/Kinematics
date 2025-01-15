#Imports



#Question 1
#Affichage de la photo du robot
#TODO Ajouter une photo avec les informations sur les joints
def show_image(image_path, title='Image'):
    image = plt.imread(image_path)
    plt.imshow(image)
    plt.axis('off')
    plt.title(title)
    plt.show()

#Question 2
#Representation du robot en home position


#Question 3
#Parametres de DH

#Question 4
#Cinématique directe


#Question 5
#Cinématique inverse