#!/usr/bin/env python
# _*_ coding: UTF-8 _*_

import rospy
from tic_tac_drone.msg import MakeMove
from geometry_msgs.msg import Point


class Algorithm():
    board = [[-1 for j in range(3)] for i in range(3)]  # stvaranje ploče - ploča[red][stupac]
    NUMWIN = 8
    win = [[0 for i in range(NUMWIN)] for j in range(2)]  # win liste

    """
    Dvije liste koje bilježe napredak svakog igrača
    U križić-kružiću je moguće pobijediti na osam načina:
        3 puna retka
        3 puna stupca
        2 dijagonale
    Svaki element liste predstavlja jednu od pobjeda
    """

    def update_win(self, player, row, col):
        """
        Funkcija koja ažurira win liste za pojedini potez. Poziva se nakon svakog pozeza
        """

        self.win[player][row] += 1
        self.win[player][3 + col] += 1
        if row == col:
            self.win[player][6] += 1

        for i in range(3):
            if row + i == 2 and col == i:
                self.win[player][7] += 1

    def print_board(self):
        """
           Ispisuje trenutno stanje igre. Elementi matrice:
               -1 - prazno polje
                0 - kružić
                1 - križić
        """

        out = ''
        for i in range(3):  # iteracija po recima

            out += '{0}'.format(i + 1)  # upis broja retka u string
            for j in range(3):  # iteracija po stupcima

                out += '['

                if self.board[i][j] == -1:  # upis stanja polja
                    out += ' '
                if self.board[i][j] == 0:
                    out += 'O'
                if self.board[i][j] == 1:
                    out += 'X'
                out += ']'
            # redak ispisan
            out += '\n'

        # ploča je u stringu, dodavanje slova stupaca
        out += '  a  b  c\n'
        rospy.loginfo(out)

    def check_board(self):
        """
        Provjerava je li netko pobijedio. Vraća:
            0 - kružić
            1 - križić
           -1 - izjednačeno ili nije gotovo
        """
        # reminder: board[col][row]
        for i in range(self.NUMWIN):

            if self.win[0][i] == 3:
                return 0
            if self.win[1][i] == 3:
                return 1

        return -1

    def check_board_move(self, row, col, player):
        """
        Provjerava je li zadani potez legalan
        :return:
            1 - potez je legalan
            0 - željeno polje je zauzeto
           -1 - željenog polja nema na ploči
           -2 - željeni igrač ne postoji
        """
        if player != 1 and player != 0:
            return -2

        if row < 3 and row >= 0 and col < 3 and col >= 0:
            field = self.board[row][col]
            if field != -1:
                return 0
            else:
                return 1
        else:
            return -1

    def call_UAV_move(self, row, col, player):
        """
        Provjerava je li moguće napraviti željeni potez ovisno o trenutnom stanju ploče, te šalje koordinate
        čvoru za prihvat točaka u topicu MakeMove.
        Vraća:
            1 - znak je uspješno stavljen na željeno polje
            0 - željeno polje je zauzeto
        """
        if self.check_board_move(row, col, player) == 1:
            # Setting message parameters
            self.make_move_msg.player = player
            self.make_move_msg.row = row
            self.make_move_msg.col = col
            self.pub.publish(self.make_move_msg)
            return 1
        else:
            return 0

    def make_board_move(self, row, col, player):
        """
        Samo stavlja na ploču znak player, u red row i stupac col.
        Poziva funkciju za ažuriranje win lista.
        Ne objavljuje koordinate.
        Poziva funkciju za ažuriranje win lista.
        Vraća:
            1 - znak je uspješno stavljen na željeno polje
            0 - željeno polje je zauzeto
           -1 - željenog polja nema na ploči
           -2 - željeni igrač ne postoji
        """
        return_value = self.check_board_move(row, col, player)

        if return_value == 1:
            self.board[row][col] = player
            self.update_win(player, row, col)
        return return_value

    def block(self, player, i):
        """
        Funkcija koju AI poziva kad primijeti da nekome nedostaje jedan
        znak za pobjedu.
        Argumenti:
            player - znak koji je potrebno staviti
            i - indeks elementa u win listi
        Vraća:
            1 ako je akcija uspješna
            0 inače
        """
        # check if the sent win is a full row
        if i in range(3):
            for j in range(3):
                if self.call_UAV_move(i, j, player):
                    return 1

        # check if the sent win is a full column
        check = [3, 4, 5]
        if check.count(i):
            for j in range(3):
                if self.call_UAV_move(j, i - 3, player):  # 3,4,5 > 0,1,2
                    return 1

        # diagonals:
        if i == 6:  # primary
            for j in range(3):
                if self.call_UAV_move(j, j, player):
                    return 1

        for j in range(3):  # secondary
            if self.call_UAV_move(2 - j, j, player):
                return 1
        return 0

    def AIX(self, moves):
        """
        AI za križić
        Argumenti:
            moves: broj trenutnog poteza u igri
        """
        player = 1

        # prvi potez -> u kut
        if moves == 0:
            self.call_UAV_move(0, 0, player)

        elif moves == 2:
            if self.win[0][1] == 1 and self.win[0][4] == 1:  # protivnik odigrao centar
                self.call_UAV_move(2, 2, player)  # igraj suprotni kut

            else:  # protivnik odigrao kut ili rub
                if self.win[0][6] == 1 or self.win[0][7] == 1:  # protivnik igrao kut
                    if not self.call_UAV_move(2, 2, player):  # probaj u kut nasuprot svojeg
                        # zauzeto, igraj u donji lijevi
                        self.call_UAV_move(2, 0, player)
                else:  # protivnik na rubu -> igraj centar
                    self.call_UAV_move(1, 1, player)

        else:  # ključni potezi odigrani, sad se napadaj i brani kad treba

            # provjeri možeš li pobijediti
            for i in range(self.NUMWIN):
                if self.win[1][i] == 2 and self.win[0][i] == 0:  # fali ti jedan, igraj tamo
                    self.block(player, i)
                    return 1

            # još ne može, brani se ako treba
            for i in range(self.NUMWIN):
                if self.win[0][i] == 2 and self.win[1][i] == 0:  # BLOCK!
                    self.block(player, i)
                    return 1

            # nijedno od toga -> probaj vući nešto agresivno
            # kutevi preko winova: 0,3,6; 0,5,7; 2,3,6; 2,5,7;
            if (self.win[1][0] + self.win[1][3] + self.win[1][6] >= 2
                and self.win[0][0] + self.win[0][3] + self.win[0][6] <= 1):
                if self.call_UAV_move(0, 0, 1):
                    return 1
            if (self.win[1][0] + self.win[1][5] + self.win[1][7] >= 2
                and self.win[0][0] + self.win[0][5] + self.win[0][7] <= 1):
                if self.call_UAV_move(0, 2, 1):
                    return 1
            if (self.win[1][2] + self.win[1][3] + self.win[1][7] >= 2
                and self.win[0][2] + self.win[0][3] + self.win[0][7] <= 1):
                if self.call_UAV_move(2, 0, 1):
                    return 1
            if (self.win[1][2] + self.win[1][5] + self.win[1][6] >= 2
                and self.win[0][2] + self.win[0][5] + self.win[0][6] <= 1):
                if self.call_UAV_move(2, 2, 1):
                    return 1

            # nasumičan potez jer je dalje izjednačeno

            for i in range(self.NUMWIN):
                if self.win[0][i] + self.win[1][i] < 3:
                    if self.block(1, i):
                        return 1

    def AIO(self, moves):
        """
        Vuče kružićev potez.
        Argumenti:
            moves - broj trenutnog poteza
        """
        player = 0

        if moves == 1:  # igraj centar ako možeš, gornji lijevi kut ako ne ide
            if not self.call_UAV_move(1, 1, 0):
                self.call_UAV_move(0, 0, 0)

        elif moves == 3:
            # provjeri jesu li oba križića na dijagonali, to je opasno

            if self.win[1][6] == 2 or self.win[1][7] == 2:  # jesu
                if self.win[0][6] == 1 and self.win[0][7] == 1:  # Ja sam u sredini -> igram rub
                    if not self.call_UAV_move(0, 1, 0):
                        self.call_UAV_move(1, 0, 0)
                else:  # Ja sam u kutu -> igram kut
                    if not self.call_UAV_move(0, 2, 0):
                        self.call_UAV_move(2, 0, 0)

            else:  # križići nisu na dijagonali

                # def
                for i in range(self.NUMWIN):
                    if self.win[1][i] == 2 and self.win[0][i] == 0:  # BLOCK!
                        self.block(player, i)
                        return 1

                # nema se što blokirati, nešto se sprema!
                # provijeri koji su kutevi opasni
                # kutevi: 0,3; 0,5; 2,3; 2,5;
                if self.win[1][0] + self.win[1][3] == 2:
                    if self.call_UAV_move(0, 0, 0):
                        return 1
                if self.win[1][0] + self.win[1][5] == 2:
                    if self.call_UAV_move(0, 2, 0):
                        return 1
                if self.win[1][2] + self.win[1][3] == 2:
                    if self.call_UAV_move(2, 0, 0):
                        return 1
                if self.win[1][2] + self.win[1][5] == 2:
                    if __name__ == '__main__':
                        if self.call_UAV_move(2, 2, 0):
                            return 1

                # nema opasnih kuteva, križić vjerojatno napravio grešku
                # gađaj neki kut
                if not self.call_UAV_move(0, 0, 0):
                    if not self.call_UAV_move(0, 2, 0):
                        self.call_UAV_move(2, 0, 0)

        else:  # Ključni potezi odigrani, samo blokiraj do kraja

            # Osim ako možeš pobijediti!
            for i in range(self.NUMWIN):
                if self.win[0][i] == 2 and self.win[1][i] == 0:  # aw yiss
                    self.block(player, i)
                    return 1

                    # ništa od pobjede
            for i in range(self.NUMWIN):
                if self.win[1][i] == 2 and self.win[0][i] == 0:  # BLOCK!
                    self.block(player, i)
                    return 1

            # Nema se što blokirati, provjeravaj kuteve
            if self.win[1][0] + self.win[1][3] == 2:
                if self.call_UAV_move(0, 0, 0):
                    return 1
            if self.win[1][0] + self.win[1][5] == 2:
                if self.call_UAV_move(0, 2, 0):
                    return 1
            if self.win[1][2] + self.win[1][3] == 2:
                if self.call_UAV_move(2, 0, 0):
                    return 1
            if self.win[1][2] + self.win[1][5] == 2:
                if self.call_UAV_move(2, 2, 0):
                    return 1

            # nasumični potez
            for i in range(self.NUMWIN):
                if self.win[0][i] + self.win[1][i] < 3:
                    if self.block(0, i):
                        return 1

    def __init__(self):
        """
        Sets up the ros publishers and subscribers.
        Published message:
            Make_move: sends the board coordinates of the next call_UAV_move to be made by defining the row and collumn
        Subscribed to the message:
            Get_move: gets information about the opponents call_UAV_move in order to update the board.
        :return:
            Void
        """
        self.pub = rospy.Publisher("Make_move", MakeMove, queue_size=1)
        self.make_move_msg = MakeMove()

if __name__ == '__main__':

    rospy.init_node("tic_tac_algo")
    ttt = Algorithm()
    human = 1  # Pretpostavka da je X
    computer = 0 if human == 1 else 1  # unos AI igrača ovisno o čovjeku

    victory = -1
    moves = 0
    player = 1
    ttt.print_board()

    while moves < 9:

        if player != human:  # potez računala
            ttt.AIX(moves) if computer == 1 else ttt.AIO(moves)
        else:
            raw_input("Čekam topic, molim Vas pritisnite ENTER kad ste spremni")

        if moves >= 4:  # Najraniji potez u kojem je moguće pobijediti
            victory = ttt.check_board()

        if victory == 1:
            ttt.print_board()
            rospy.loginfo("Čestitke križiću! Pobijedio je u ovoj rundi!")
            break

        if victory == 0:
            ttt.print_board()
            rospy.loginfo("Čestitke kružiću! Pobijedio je u ovoj rundi!")
            break

        moves += 1
        player = 0 if player == 1 else 1  # zamjena igrača
        ttt.print_board()

    if victory == -1:

        # ploča popunjena

        victory = ttt.check_board()
        if victory == 1:
            rospy.loginfo("Čestitke križiću! Pobijedio je u ovoj rundi!")

        elif victory == 0:
            rospy.loginfo("Čestitke kružiću! Pobijedio je u ovoj rundi!")

        else:
            rospy.loginfo("Izjednačeno je!")