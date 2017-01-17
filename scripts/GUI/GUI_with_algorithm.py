#!/usr/bin/env python
#  _*_ coding: UTF-8 _*_

import rospy
from Tkinter import *
from algorithm import Algorithm # klasa algoritma
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from tic_tac_drone.msg import CustomPose
import os


__metaclass__ = type


class Program(Frame):
    CANVAS_SIZE = 18
    PANIC_BUTTON_SIZE = 18

    def __init__(self, master=None):

        # VAR
        self.victory = -1
        self.moves = 0
        self.game_running = 0

        Frame.__init__(self, master)
        self.config(width=1000, height=1000)
        self.grid(rows=100, columns=100, sticky=N)
        self.kreiraj_sucelje()
        self.canvas = Canvas(self, width=300, height=300)
        self.canvas.grid(row=1, column=1, rowspan=self.CANVAS_SIZE, columnspan=self.CANVAS_SIZE)
        self.canvas.create_line(0, 100, 300, 100)
        self.canvas.create_line(0, 200, 300, 200)
        self.canvas.create_line(100, 0, 100, 300)
        self.canvas.create_line(200, 0, 200, 300)

        rospy.Subscriber("MyUAV/cpose", CustomPose, self.get_pose)
        self.uav_pos = CustomPose()

        self.pub_r = rospy.Publisher("reference", Point, queue_size=1)
        self.pub_p = rospy.Publisher("kill", Bool, queue_size=1)
        self.reference = Point()
        self.kill = Bool()
        self.kill = 1
        return

    def kreiraj_sucelje(self):
        # Fonts used
        f1 = ('Calibri', 14, 'bold')
        f2 = ('Calibri', 12)

        # Images used
        image_path = os.path.join(os.path.expanduser('~'), 'catkin_ws', 'src', 'tic_tac_drone', 'scripts', 'GUI')
        image_panic_button = PhotoImage(file=os.path.join(image_path, 'panic2.ppm'))
        image_under_construction = PhotoImage(file=os.path.join(image_path, 'underconstruction.ppm'))

        # Panic Button
        self.panic = Button(self, height=300, width=300, image=image_panic_button, command=self.panic_callback)
        self.panic.grid(row=self.CANVAS_SIZE + 2, column=2 * self.CANVAS_SIZE + 1, rowspan=self.PANIC_BUTTON_SIZE,
                        columnspan=self.PANIC_BUTTON_SIZE)
        self.panic.image = image_panic_button

        # Mode select
        self.mode = Label(self, font=f1, text="MODE")
        self.mode.grid(row=1, column=self.CANVAS_SIZE + 1, sticky=N)
        modevar = StringVar()
        self.mode_sel = OptionMenu(self, modevar, "AUTO", "SEMI", "MAN")
        modevar.set("AUTO")
        self.mode_sel.config(width=5)
        self.mode_sel.grid(row=2, column=self.PANIC_BUTTON_SIZE + 1, sticky=N)

        # Game control
        self.start_next = Button(self, text="START GAME / NEXT MOVE")
        self.start_next.grid(row=1, column=self.CANVAS_SIZE + 2, rowspan=6, sticky=N, pady=4)
        self.reset = Button(self, text="RESTART GAME")
        self.reset.grid(row=2, column=self.CANVAS_SIZE + 2, rowspan=6, sticky=N)
        self.end = Button(self, text="END GAME")
        self.end.grid(row=3, column=self.CANVAS_SIZE + 2, rowspan=6, sticky=N)

        # Position input
        self.position = Label(self, font=f2, text="Position:")
        self.x_label = Label(self, font=f2, text="X:")
        self.y_label = Label(self, font=f2, text="Y:")
        self.z_label = Label(self, font=f2, text="Z:")
        self.position.grid(row=1, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 2)
        self.x_label.grid(row=2, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 1)
        self.y_label.grid(row=3, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 1)
        self.z_label.grid(row=4, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 1)

        self.x1 = StringVar(self, value='0')
        self.x_position = Entry(self, width=10, font=f2, textvariable=self.x1)
        self.x_position.grid(row=2, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 2)

        self.y1 = StringVar(self, value='0')
        self.y_position = Entry(self, width=10, font=f2, textvariable=self.y1)
        self.y_position.grid(row=3, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 2)

        self.z1 = StringVar(self, value='0')
        self.z_position = Entry(self, width=10, font=f2, textvariable=self.z1)
        self.z_position.grid(row=4, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 2)

        self.submit = Button(self, text="Submit", command = self.send_reference)
        self.submit.grid(row=5, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 2)

        # Player select
        self.play_label = Label(self, font=f2, text="Player")
        self.play_label.grid(row=6, column=self.CANVAS_SIZE + 2)
        play_x_o = StringVar()
        self.play_x = Radiobutton(self, font=f2, variable=play_x_o, text="X", value="X")
        self.play_o = Radiobutton(self, font=f2, variable=play_x_o, text="O", value="O")
        self.play_x.grid(row=7, column=self.CANVAS_SIZE + 2)
        self.play_o.grid(row=8, column=self.CANVAS_SIZE + 2)

        # Signals and parameters
        self.odzivi = Label(self, font=f2, text="Signals:")
        self.parametri = Label(self, font=f2, text="Parameters:")

        self.odzivi.grid(row=self.CANVAS_SIZE + 1, column=self.CANVAS_SIZE / 2)
        self.parametri.grid(row=self.CANVAS_SIZE + 1, column=self.CANVAS_SIZE + 1)

        # Under Construction
        self.uc1 = Button(self, image=image_under_construction)
        self.uc1.image = image_under_construction
        self.uc1.grid(row=self.CANVAS_SIZE + 2, column=1, rowspan=self.CANVAS_SIZE, columnspan=self.CANVAS_SIZE)
        self.uc2 = Button(self, image=image_under_construction)
        self.uc2.image = image_under_construction
        self.uc2.grid(row=self.CANVAS_SIZE + 2, column=self.CANVAS_SIZE + 1, rowspan=self.CANVAS_SIZE,
                      columnspan=self.CANVAS_SIZE)

        # Current position
        self.current_position = Label(self, font=f2, text="Current position:")
        self.current_x = Label(self, font=f2, text="X:")
        self.current_y = Label(self, font=f2, text="Y:")
        self.current_z = Label(self, font=f2, text="Z:")
        self.current_position.grid(row=1, column=self.CANVAS_SIZE + 5 + self.CANVAS_SIZE)
        self.current_x.grid(row=2, column=self.CANVAS_SIZE + 4 + self.CANVAS_SIZE)
        self.current_y.grid(row=3, column=self.CANVAS_SIZE + 4 + self.CANVAS_SIZE)
        self.current_z.grid(row=4, column=self.CANVAS_SIZE + 4 + self.CANVAS_SIZE)
        return

    def crtaj_plocu(self):
        """
        Crta simbole po ploči za igru.

        Čita trenutno stanje ploče u objektu algoritma i ovisno o njemu stavlja simbole
        u pojedina polja ploče.

        Ploča u algoritmu organizirana je na sljedeći način:
        0[][][]
        1[][][]
        2[][][]
         0 1 2

        Primjer dohvata:
         ttt.board[red][stupac]

        Vrijednosti polja:
            -1 -> prazno polje
             0 -> kružić
             1 -> križić

        :return:
        None
        """
        for row in range (0, 2):
            for col in range (0, 2):
                centar_x=row*100+50
                centar_y=col*100+50
                if ttt.board[row][col]==0:
                    self.canvas.create_oval(centar_x-25, centar_y-25, centar_x+25, centar_y+25, width=3, outline='red')
                if ttt.board[row][col]==1:
                    self.canvas.create_line(centar_x-25, centar_y-25, centar_x+25, centar_y+25, width=3, fill='gray')
                    self.canvas.create_line(centar_x-25, centar_y+25, centar_x+25, centar_y-25, width=3, fill='gray')


    def start_or_next_callback(self):
        """
        Funkcija gumba start game/next move

        Pri prvom pritisku setira varijable u objektu algoritma:

        inače poziva odgovarajuću funkciju (AIX ili AIO) iz istog objekta kako bi
        se odabralo novo polje za slijetanje


        :return:
        None
        """

        if not (self.play_x_o.get() == 'X' or self.play_x_o.get() == 'O') and self.game_running == 0:
            tkMessageBox.showinfo("Error", "Select a player...")
            return

        if not self.game_running:
            ttt.board = [[-1 for j in range(3)] for i in range(3)]
            ttt.win = [[0 for i in range(ttt.NUMWIN)] for j in range(2)]
            self.game_running = 1
            # TODO
            # još neke stvari, nemrem se setit
        else:
            pass
            # pozvati potez ovisno o varijabli radio buttona

            # pozvati provjeru pobjede:
            # self.victory = ttt.check_board()
            # ako smo pobijedili: memefest

    def send_reference(self):
        self.reference.x = float(self.x1.get())
        self.reference.y = float(self.y1.get())
        self.reference.z = float(self.z1.get())

        self.pub_r.publish(self.reference)

    def get_pose (self, data):
        self.uav_pos.x = data.x
        self.uav_pos.y = data.y
        self.uav_pos.z = data.z

    def restart_callback(self):
        """
        Funkcija gumba restart

        Vraća sve na početak i ponovno započinje igru
        Elegantno rješenje: self.game_running = 0 i pozvat start_or_next_callback()

        :return:
        None
        """
        self.game_running = 0
        return self.start_or_next_callback


    def end_callback(self):
        """
        Završava trenutnu igru

        Sve stavlja u nulu

        :return:
        None
        """
        self.game_running = 0


    def panic_callback(self):
        """
        Letjelica se gasi i pada na tlo

        Black magic boi

        :return:
        None
        """
        self.pub_p.publish(self.kill)

    def Submit_callback(self):
        """
        Šalje koordinate u prostoru letjelici

        Publisha topic regulatoru
        :return:
        None
        """

# TODO
#   Provjeru pobjede
#   Logika s modom rada!
#       Ne dozvoliti postavljanje fiksnih koordinata ako si u automatskom i tak...
#   Čekanje poteza
#       Ne dati next_move dok čekaš protivnikov potez
#       Baciti messagebox
#   Gumb osvježi polja
#   Prikaz vrijednosi iz optitracka


if __name__ == '__main__':
    # stvaranje nodea
    rospy.init_node("GUI")

    # stvaranje objekta algoritma koji:
    #   rukuje topicima
    #   prati stanje igre
    #   vuče poteze

    ttt = Algorithm()

    # stvaranje objekta za sučelje

    p = Program()
    p.master.title('Tic-Tac-Drone-GUI')

    # zavrti petlju za rukovanje događajima
    # nije potreban rospy.spin()?
    mainloop()




