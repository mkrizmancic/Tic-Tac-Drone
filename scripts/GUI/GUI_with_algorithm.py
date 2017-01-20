#!/usr/bin/env python
#  _*_ coding: UTF-8 _*_

import rospy
from Tkinter import *
from algorithm import Algorithm  # klasa algoritma
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from tic_tac_drone.msg import CustomPose
from tic_tac_drone.msg import MakeMove
import os
import tkMessageBox

__metaclass__ = type


class Program(Frame):
    CANVAS_SIZE = 18
    PANIC_BUTTON_SIZE = 18

    def __init__(self, master=None):

        Frame.__init__(self, master)
        self.config(width=1000, height=1000)
        self.grid(rows=100, columns=100, sticky=N)

        # VAR
        self.victory = -1
        self.moves = 0
        self.game_running = 0
        self.uav_pos_x_str = StringVar()
        self.uav_pos_y_str = StringVar()
        self.uav_pos_z_str = StringVar()
        self.current_player = -1

        rospy.Subscriber("Get_move", MakeMove, self.read_move)
        rospy.Subscriber("MyUAV/cpose", CustomPose, self.get_pose)
        self.uav_pos = CustomPose()
        self.uav_pos.x = 0
        self.uav_pos.y = 0
        self.uav_pos.z = 0

        self.pub_r = rospy.Publisher("reference", Point, queue_size=1)
        self.pub_panic = rospy.Publisher("kill", Bool, queue_size=1)
        self.pub_MakeMove = rospy.Publisher("Make_move", MakeMove,queue_size=1)  
        self.pub_flightMode = rospy.Publisher("flight_mode", Bool, queue_size=1)
        self.reference = Point()
        self.kill = Bool()
        self.kill = 1
        self.auto_mode = Bool()

        self.kreiraj_sucelje()
        self.canvas = Canvas(self, width=300, height=300)
        self.canvas.grid(row=1, column=1, rowspan=self.CANVAS_SIZE, columnspan=self.CANVAS_SIZE)
        self.canvas.create_line(0, 100, 300, 100)
        self.canvas.create_line(0, 200, 300, 200)
        self.canvas.create_line(100, 0, 100, 300)
        self.canvas.create_line(200, 0, 200, 300)

        return

    def kreiraj_sucelje(self):
        # Fonts used
        f1 = ('Calibri', 14, 'bold')
        f2 = ('Calibri', 12)
        f3 = ('Calibri', 20)

        # Images used
        image_path = os.path.join(os.path.expanduser('~'), 'catkin_ws', 'src', 'tic_tac_drone', 'scripts', 'GUI')
        image_panic_button = PhotoImage(file=os.path.join(image_path, 'panic2.ppm'))

        # Panic Button
        self.panic = Button(self, height=300, width=300, image=image_panic_button, command=self.panic_callback)
        self.panic.grid(row=self.CANVAS_SIZE + 2, column=2 * self.CANVAS_SIZE + 1, rowspan=self.PANIC_BUTTON_SIZE,
                        columnspan=self.PANIC_BUTTON_SIZE)
        self.panic.image = image_panic_button

        # Mode select
        self.mode = Label(self, font=f1, text="MODE")
        self.mode.grid(row=1, column=self.CANVAS_SIZE + 1, sticky=N, columnspan=6)
        self.modevar = StringVar()
        self.mode_sel = OptionMenu(self, self.modevar, "AUTO", "SEMI", "MAN", command=self.change_mode)
        self.modevar.set("AUTO")
        self.mode_sel.config(width=5)
        self.mode_sel.grid(row=2, column=self.PANIC_BUTTON_SIZE + 1, sticky=N, columnspan=6)

        # Game control
        self.start_next = Button(self, text="START GAME", command=self.start_or_next_callback)
        self.start_next.grid(row=1, column=self.CANVAS_SIZE + 8, rowspan=6, sticky=N, pady=4)
        self.reset = Button(self, text="RESTART GAME", command=self.restart_callback)
        self.reset.grid(row=2, column=self.CANVAS_SIZE + 8, rowspan=6, sticky=N)
        self.end = Button(self, text="END GAME", command=self.end_callback)
        self.end.grid(row=3, column=self.CANVAS_SIZE + 8, rowspan=6, sticky=N)

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

        self.submit = Button(self, text="Submit", command=self.send_reference, state='disabled')
        self.submit.grid(row=5, column=self.CANVAS_SIZE + self.CANVAS_SIZE + 2)

        # Player select
        self.play_label = Label(self, font=f2, text="Player")
        self.play_label.grid(row=6, column=self.CANVAS_SIZE + 8)
        self.play_x_o = StringVar()
        self.play_x = Radiobutton(self, font=f2, variable=self.play_x_o, text="X", value="X")
        self.play_o = Radiobutton(self, font=f2, variable=self.play_x_o, text="O", value="O")
        self.play_x.grid(row=7, column=self.CANVAS_SIZE + 8)
        self.play_o.grid(row=8, column=self.CANVAS_SIZE + 8)

        # Current position
        self.current_position_label = Label(self, font=f2, text="Current position:")
        self.current_x = Label(self, font=f2, text="X:")
        self.current_y = Label(self, font=f2, text="Y:")
        self.current_z = Label(self, font=f2, text="Z:")
        self.current_position_label.grid(row=1, column=self.CANVAS_SIZE + 5 + self.CANVAS_SIZE)
        self.current_x.grid(row=2, column=self.CANVAS_SIZE + 4 + self.CANVAS_SIZE)
        self.current_y.grid(row=3, column=self.CANVAS_SIZE + 4 + self.CANVAS_SIZE)
        self.current_z.grid(row=4, column=self.CANVAS_SIZE + 4 + self.CANVAS_SIZE)

        # Current position display:
        self.current_position_label_x = Label(self, font=f2, textvariable=self.uav_pos_x_str)
        self.current_position_label_x.grid(row=2, column=self.CANVAS_SIZE + 5 + self.CANVAS_SIZE)

        self.current_position_label_y = Label(self, font=f2, textvariable=self.uav_pos_y_str)
        self.current_position_label_y.grid(row=3, column=self.CANVAS_SIZE + 5 + self.CANVAS_SIZE)

        self.current_position_label_z = Label(self, font=f2, textvariable=self.uav_pos_z_str)
        self.current_position_label_z.grid(row=4, column=self.CANVAS_SIZE + 5 + self.CANVAS_SIZE)

        # Opponent move input
        self.opponent_input = Label(self, font=f2, text="Opponent move input")
        self.opponent_input.grid(row=self.CANVAS_SIZE + 1, column=1, columnspan=18)

        self.opponent_move1 = Button(self, width=13, height=5, command=self.opponent_input_1, state='disabled', text='a1')
        self.opponent_move1.grid(row=self.CANVAS_SIZE + 2, column=1, columnspan=5, rowspan=5)
        self.opponent_move2 = Button(self, width=13, height=5, command=self.opponent_input_2, state='disabled', text='a2')
        self.opponent_move2.grid(row=self.CANVAS_SIZE + 2, column=7, columnspan=5, rowspan=5)
        self.opponent_move3 = Button(self, width=13, height=5, command=self.opponent_input_3, state='disabled', text='a3')
        self.opponent_move3.grid(row=self.CANVAS_SIZE + 2, column=13, columnspan=5, rowspan=5)
        self.opponent_move4 = Button(self, width=13, height=5, command=self.opponent_input_4, state='disabled', text='b1')
        self.opponent_move4.grid(row=self.CANVAS_SIZE + 8, column=1, columnspan=5, rowspan=5)
        self.opponent_move5 = Button(self, width=13, height=5, command=self.opponent_input_5, state='disabled', text='b2')
        self.opponent_move5.grid(row=self.CANVAS_SIZE + 8, column=7, columnspan=5, rowspan=5)
        self.opponent_move6 = Button(self, width=13, height=5, command=self.opponent_input_6, state='disabled', text='b3')
        self.opponent_move6.grid(row=self.CANVAS_SIZE + 8, column=13, columnspan=5, rowspan=5)
        self.opponent_move7 = Button(self, width=13, height=5, command=self.opponent_input_7, state='disabled', text='c1')
        self.opponent_move7.grid(row=self.CANVAS_SIZE + 14, column=1, columnspan=5, rowspan=5)
        self.opponent_move8 = Button(self, width=13, height=5, command=self.opponent_input_8, state='disabled', text='c2')
        self.opponent_move8.grid(row=self.CANVAS_SIZE + 14, column=7, columnspan=5, rowspan=5)
        self.opponent_move9 = Button(self, width=13, height=5, command=self.opponent_input_9, state='disabled', text='c3')
        self.opponent_move9.grid(row=self.CANVAS_SIZE + 14, column=13, columnspan=5, rowspan=5)
        return

    def draw_board(self):
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
        self.canvas.create_line(0, 100, 300, 100)
        self.canvas.create_line(0, 200, 300, 200)
        self.canvas.create_line(100, 0, 100, 300)
        self.canvas.create_line(200, 0, 200, 300)
        for row in range(0, 3):
            for col in range(0, 3):
                centar_x = col * 100 + 50
                centar_y = row * 100 + 50
                if ttt.board[row][col] == 0:
                    self.canvas.create_oval(centar_x - 25, centar_y - 25, centar_x + 25, centar_y + 25, width=3, outline='red')
                if ttt.board[row][col] == 1:
                    self.canvas.create_line(centar_x - 25, centar_y - 25, centar_x + 25, centar_y + 25, width=3, fill='gray')
                    self.canvas.create_line(centar_x - 25, centar_y + 25, centar_x + 25, centar_y - 25, width=3, fill='gray')
                if ttt.win[0][0] == 3 or ttt.win[1][0] == 3:
                    self.canvas.create_line(25, 50, 275, 50)
                if ttt.win[0][1] == 3 or ttt.win[1][1] == 3:
                    self.canvas.create_line(25, 150, 275, 150)
                if ttt.win[0][2] == 3 or ttt.win[1][2] == 3:
                    self.canvas.create_line(25, 250, 275, 250)
                if ttt.win[0][3] == 3 or ttt.win[1][3] == 3:
                    self.canvas.create_line(50, 25, 50, 275)
                if ttt.win[0][4] == 3 or ttt.win[1][4] == 3:
                    self.canvas.create_line(150, 25, 150, 275)
                if ttt.win[0][5] == 3 or ttt.win[1][5] == 3:
                    self.canvas.create_line(250, 25, 250, 275)
                if ttt.win[0][6] == 3 or ttt.win[1][6] == 3:
                    self.canvas.create_line(25, 25, 275, 275)
                if ttt.win[0][7] == 3 or ttt.win[1][7] == 3:
                    self.canvas.create_line(25, 275, 275, 25)

    def start_or_next_callback(self):
        """
        Funkcija gumba start game/next move
        Pri prvom pritisku setira varijable u objektu algoritma:
        inače poziva odgovarajuću funkciju (AIX ili AIO) iz istog objekta kako bi
        se odabralo novo polje za slijetanje
        :return:
        None
        """
        if (not (self.play_x_o.get() == 'X' or self.play_x_o.get() == 'O')
            and self.game_running == 0):
            tkMessageBox.showinfo("Error", "Please, select a player.")
            return

        if not self.game_running:
            self.canvas.delete('all')
            ttt.board = [[-1 for j in range(3)] for i in range(3)]
            ttt.win = [[0 for i in range(ttt.NUMWIN)] for j in range(2)]
            self.moves = 0
            self.game_running = 1
            self.current_player = 1
            self.draw_board()
            self.opponent_moves_visibility()
            self.start_next.config(text="NEXT MOVE")
            self.mode_sel.config(state=DISABLED)

        else:
            if self.play_x_o.get() == 'X':
                ttt.AIX(self.moves)
                self.provjeri_pobjede()
                self.draw_board()
            if self.play_x_o.get() == 'O':
                ttt.AIO(self.moves)
                self.provjeri_pobjede()
                self.draw_board()

    def send_reference(self):
        self.reference.x = float(self.x1.get())
        self.reference.y = float(self.y1.get())
        self.reference.z = float(self.z1.get())

        self.pub_r.publish(self.reference)

    def provjeri_pobjede(self):
        self.victory = ttt.check_board()
        self.draw_board()
        if self.victory == 1:
            tkMessageBox.showinfo("Kraj!", "Čestitke križiću! Pobijedio je u ovoj rundi!")
            self.end_callback()
        elif self.victory == 0:
            tkMessageBox.showinfo("Kraj!", "Čestitke kružiću! Pobijedio je u ovoj rundi!")
            self.end_callback()
        elif self.moves == 9 and self.victory == -1:
            tkMessageBox.showinfo("Kraj!", "Izjednačeno je!")
            self.end_callback()

    def get_pose(self, data):
        self.uav_pos.x = data.x
        self.uav_pos.y = data.y
        self.uav_pos.z = data.z
        self.uav_pos_x_str.set("{:1.3f}".format(data.x))
        self.uav_pos_y_str.set("{:1.3f}".format(data.y))
        self.uav_pos_z_str.set("{:1.3f}".format(data.z))
        self.draw_board()
        p.update_idletasks()

    def restart_callback(self):
        """
        Funkcija gumba restart
        Vraća sve na početak i ponovno započinje igru
        Elegantno rješenje: self.game_running = 0 i pozvat start_or_next_callback()
        :return:
        None
        """
        self.end_callback()
        self.start_or_next_callback()

    def end_callback(self):
        """
        Završava trenutnu igru
        Sve stavlja u nulu
        :return:
        None
        """
        self.game_running = 0
        ttt.board = [[-1 for j in range(3)] for i in range(3)]
        ttt.win = [[0 for i in range(ttt.NUMWIN)] for j in range(2)]
        self.moves = 0
        self.victory = -1
        self.draw_board()
        self.start_next.config(text="START GAME")
        self.mode_sel.config(state=ACTIVE)

    def panic_callback(self):
        """
        Letjelica se gasi i pada na tlo
        :return:
        None
        """
        self.pub_panic.publish(self.kill)

    def change_mode(self, mod):
        if (mod == 'AUTO'):
            self.auto_mode = True
            self.submit.config(state=DISABLED)
            self.start_next.config(state=ACTIVE)
            self.end.config(state=ACTIVE)
            self.reset.config(state=ACTIVE)
        elif (mod == 'SEMI'):
            self.auto_mode = True
            self.submit.config(state=ACTIVE)
            self.start_next.config(state=DISABLED)
            self.end.config(state=DISABLED)
            self.reset.config(state=DISABLED)
        else:
            self.auto_mode = False
            self.submit.config(state=DISABLED)
            self.start_next.config(state=DISABLED)
            self.end.config(state=DISABLED)
            self.reset.config(state=DISABLED)

        self.pub_flightMode.publish(self.auto_mode)

    # unos protivničkog poteza

    def opponent_input_1(self):
        if ttt.board[0][0] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[0][0] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 0, 0)
            if self.play_x_o.get() == 'O':
                ttt.board[0][0] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 0, 0)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def opponent_input_2(self):
        if ttt.board[0][1] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[0][1] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 0, 1)
            if self.play_x_o.get() == 'O':
                ttt.board[0][1] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 0, 1)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def opponent_input_3(self):
        if ttt.board[0][2] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[0][2] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 0, 2)
            if self.play_x_o.get() == 'O':
                ttt.board[0][2] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 0, 2)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def opponent_input_4(self):
        if ttt.board[1][0] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[1][0] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 1, 0)
            if self.play_x_o.get() == 'O':
                ttt.board[1][0] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 1, 0)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def opponent_input_5(self):
        if ttt.board[1][1] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[1][1] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 1, 1)
            if self.play_x_o.get() == 'O':
                ttt.board[1][1] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 1, 1)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def opponent_input_6(self):
        if ttt.board[1][2] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[1][2] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 1, 2)
            if self.play_x_o.get() == 'O':
                ttt.board[1][2] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 1, 2)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def opponent_input_7(self):
        if ttt.board[2][0] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[2][0] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 2, 0)
            if self.play_x_o.get() == 'O':
                ttt.board[2][0] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 2, 0)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def opponent_input_8(self):
        if ttt.board[2][1] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[2][1] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 2, 1)
            if self.play_x_o.get() == 'O':
                ttt.board[2][1] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 2, 1)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def opponent_input_9(self):
        if ttt.board[2][2] == -1:
            if self.play_x_o.get() == 'X':
                ttt.board[2][2] = 0
                self.moves += 1
                self.draw_board()
                ttt.update_win(0, 2, 2)
            if self.play_x_o.get() == 'O':
                ttt.board[2][2] = 1
                self.moves += 1
                self.draw_board()
                ttt.update_win(1, 2, 2)
            self.provjeri_pobjede()
            self.switch_players()
            return
        else:
            tkMessageBox.showinfo("Error", "Field is occupied...")
            pass

    def read_move(self, data):
        self.moves += 1
        ttt.update_win(data.player, data.row, data.col)
        print
        data
        ttt.board[data.row][data.col] = data.player
        self.switch_players()

    def switch_players(self):
        self.current_player = 1 if self.current_player == 0 else 0
        self.opponent_moves_visibility()
        self.provjeri_pobjede()

    def opponent_moves_visibility(self):
        if (self.current_player == 1 and self.play_x_o.get() == 'X'
            or self.current_player == 0 and self.play_x_o.get() == 'O'):
            self.opponent_move1.config(state='disabled')
            self.opponent_move2.config(state='disabled')
            self.opponent_move3.config(state='disabled')
            self.opponent_move4.config(state='disabled')
            self.opponent_move5.config(state='disabled')
            self.opponent_move6.config(state='disabled')
            self.opponent_move7.config(state='disabled')
            self.opponent_move8.config(state='disabled')
            self.opponent_move9.config(state='disabled')
        else:
            self.opponent_move1.config(state='active')
            self.opponent_move2.config(state='active')
            self.opponent_move3.config(state='active')
            self.opponent_move4.config(state='active')
            self.opponent_move5.config(state='active')
            self.opponent_move6.config(state='active')
            self.opponent_move7.config(state='active')
            self.opponent_move8.config(state='active')
            self.opponent_move9.config(state='active')

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