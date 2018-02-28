# -*- coding: utf-8 -*-
"""
Created on Mon Jan  9 09:33:16 2017

@author: kevin
"""

import matplotlib.pyplot as plt
import numpy as np
import os 

tabResultatAllITE = {}
tabResultatAllCBG = {}

directory = os.getcwd() + "/IAR_SEHNOUNI_TEBIB/Les tests IAR/"


directoryITE = []
directoryCBG = []
    
def lireFichierResultat(filename, solo):
    fichier = open(filename, "r")
    contenu = fichier.read()
    contenu = contenu.split("\n")
    tabResultatAux = []
    for ligne in contenu:
        tabResultatAux.append(ligne.split(";"))
    fichier.close()
    del tabResultatAux[-1]
    
    if solo == 1:
        return tabResultatAux
    
    if len(filename.split("ITE")) > 1:
        tabResultatAllITE[filename] = tabResultatAux
    elif len(filename.split("CBG")) > 1:      
        tabResultatAllCBG[filename] = tabResultatAux

def courbeEnergieEnergiePotentielle(tabResultatRecuperer):
    xTime = tabResultatRecuperer[:,0].astype(np.float)
    yE = tabResultatRecuperer[:,1].astype(np.float)
    yEp = tabResultatRecuperer[:,2].astype(np.float)
    plt.axis([150,300 , 0, 1.2])
    
    plt.title("Figure B.6.1")
    plt.yscale('linear')
    
    p1 = plt.plot(xTime,yE)
    p2 = plt.plot(xTime,yEp)
    
    plt.xlabel("Time")
    plt.ylabel("Energie et EnergiePotentielle")
    plt.show()



def courbeSelectionAction(tabResultatRecuperer):
    xTime = tabResultatRecuperer[:,0].astype(np.float)
    listeActionAux = tabResultatRecuperer[:,3]
    listeAction = []
    for i in listeActionAux:
        listeAction.append(i.split('\r')[0])
    
    plt.title("Figure B.6.2")
    plt.axis([150, 300, -0.1, 6.1])
    yAction = []
    for action in listeAction:
        if action == 'ApproachEp':
            yAction.append(6)
        elif  action == 'ApproachE':
            yAction.append(5)
        elif action == 'AvoidOstacle':
            yAction.append(4)
        elif action == 'Rest':
            yAction.append(3)
        elif action == 'Wander':
            yAction.append(2)
        elif action == 'ReloadOnEp':
            yAction.append(1)
        elif action == 'ReloadOnE':
            yAction.append(0)

    plt.plot(xTime, yAction)
    plt.xlabel("Time")
    plt.ylabel("Action")
    plt.show()

def histogrammeEnergie(tabResultat):
    aux = []
    for i in tabResultat:
        for j in range(len(tabResultat[i])):
            aux.append(tabResultat[i][j])
    
    tabResultatRecuperer = np.array(aux)
    
    xEnergie = tabResultatRecuperer[:,1].astype(np.float)
    somme = np.sum(xEnergie)    
    plt.hist(xEnergie, normed=True)
    plt.axis([0,1,0.0,6])
    plt.xlabel("Energie")
    plt.show()
    
def histogrammeEnergiePotentielle(tabResultat):
    aux = []
    for i in tabResultat:
        for j in range(len(tabResultat[i])):
            aux.append(tabResultat[i][j])
    
    tabResultatRecuperer = np.array(aux)
    xEnergie = tabResultatRecuperer[:,2].astype(np.float)
    somme = np.sum(xEnergie)
        
    plt.hist(xEnergie, normed=True)
    plt.axis([0,1,0.0,6])
    plt.xlabel("EnergiePotentiel")
    plt.show()
    
def histogrammePotentialEnergieConsumptionRate(tabResultat):
    tableauRange = np.arange(0.085, 0.135, 0.005)
    DicoAux = {}
    tabToto = []
    
    for i in tableauRange:
        DicoAux[round(i,4)] = 0
   
    somme = 0
    for i in tabResultat:
        j =  np.array(tabResultat[i])
        if len(j) > 0:
            aux = j[:,2].astype(np.float)
            somme += sum(aux)
               
    moyenne = somme / len(tabResultat.keys())
    
    for i in tabResultat:
        j =  np.array(tabResultat[i])
        if len(j) > 0:
            aux = j[:,0].astype(np.float)
            tabToto.append(moyenne / aux[-1])
            
    tabFinal = []
    for i in tableauRange:
        for k in range(DicoAux[round(i,3)]):
            tabFinal.append(i)
    
    plt.hist(tabToto, normed=True)
    plt.xlabel("EP/s")
    #plt.axis([0.085,0.135,0.0,1000])
    plt.show()
   
if __name__ == '__main__':

    #Chemin relatif au fichier utilisé pour obtenir les graphes courbeEnergieEnergiePotentielle et courbeSelectionAction
    name = "Test5 - OK/ITE/TestSerie5Test11.txt"
    
    #On remplit les tableaux contenant le path de tous les fichiers de résultat
    for i in range(0,20):
        directoryITE.append(directory+"Test"+str(i+1)+" - OK/ITE/")
        directoryCBG.append(directory+"Test"+str(i+1)+" - OK/CBG/")    

    #On lit tous les fichiers
    for j in directoryITE:
        for i  in os.listdir(j):
            lireFichierResultat(j+i,0)

    for j in directoryCBG:
        for i  in os.listdir(j):
            lireFichierResultat(j+i,0)
    
    #On passe tous les tableaux en np.array pour pouvoir recuperer les colonnes des tableaux facilement par la suite            
    for i in tabResultatAllITE:
        tabResultatAllITE[i] = np.array(tabResultatAllITE[i])
    for i in tabResultatAllCBG:
        tabResultatAllCBG[i] = np.array(tabResultatAllCBG[i])
        
    #Calcul du temps moyen pour ITE        
    sommeTemps = 0
    nbITE = 0
    for i in tabResultatAllITE:
        tab = tabResultatAllITE[i]
        if len(tab) > 0:
            tab = tab[:,0].astype(np.float)
            sommeTemps += tab[-1]
            nbITE += 1
    
    print "La valeur moyenne pour ITE est de " + str(sommeTemps / nbITE) + "s."

    #Calcul du temps moyen pour CBG
    sommeTemps = 0
    nbCBG = 0
    for i in tabResultatAllCBG:
        tab = tabResultatAllCBG[i]
        
        if len(tab) > 0:
            tab = tab[:,0].astype(np.float)
            sommeTemps += tab[-1]
            nbCBG += 1
    
    print "La valeur moyenne pour CBG est de " + str(sommeTemps / nbCBG) + "s."
    
    #Affichage de la courbe de l'energie et de l'energie potentielle au cours du temps
    courbeEnergieEnergiePotentielle(np.array(lireFichierResultat(directory+name, 1))) 
    
    #Affichage de la courbe selection du temps au cours du temps
    #courbeSelectionAction(np.array(lireFichierResultat(directory+name,1)))

    #Histogramme de l'energie pour l'ITE
    #histogrammeEnergie(tabResultatAllITE)   
    
    #Histogramme de l'energie pour CBG
    #histogrammeEnergie(tabResultatAllCBG)     
    
    #histogramme de l'energie potentielle pour l'ITE
    #histogrammeEnergiePotentielle(tabResultatAllITE)    
    
    #histogramme de l'energie potentielle pour le CBG
    #histogrammeEnergiePotentielle(tabResultatAllCBG)
    
    #histogramme de la consommation de l'energie potentielle par seconde
    #histogrammePotentialEnergieConsumptionRate(tabResultatAllCBG)