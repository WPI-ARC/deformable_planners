#!/usr/bin/python
'''
Created on Jul 20, 2011

@author: Calder
'''
import Tkinter
from Tkinter import *
import Tkconstants
import tkFileDialog
import tkMessageBox
import sys
import os

class SEARCH_GUI(Tkinter.Frame):
    '''
    classdocs
    '''


    def __init__(self, root):
        '''
        Constructor
        '''
        Tkinter.Frame.__init__(self, root)
        # Design checkboxes
        self.pixelBoxes = []
        self.pixelVars = []
        width = 23
        depth = 18
        for i in range(width):
            for j in range(depth):
                tempCBVar = IntVar()
                self.pixelVars.append(tempCBVar)
                tempCB = Checkbutton(self, text="", variable=tempCBVar)
                tempCB.grid(row=j, column=i)
                self.pixelBoxes.append(tempCB)
        
        # button
        self.processbutton = Tkinter.Button(self, text='Process...', command=self.askprocess).grid(row=21, columnspan=10)

    def askprocess(self):
        print "Processing"

if __name__=='__main__':
    root = Tkinter.Tk()
    root.title("DVXL generator")
    SEARCH_GUI(root).pack()
    root.mainloop()
        
