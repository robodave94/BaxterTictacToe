#!/usr/bin/python

#MEMO, capture bw image as tempalte of 'x' with arm slot_location_x+14cm
import threading
import numpy as np
import os
import sys
from cv_bridge import CvBridge

import random
import copy

import matplotlib.pylab as plt
import matplotlib.cm as cm

import math

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from baxter_core_msgs.msg import EndpointState
import message_filters
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import String
import rospy
import signal


class GameEngine:
    
    def __init__(self):
        
        self.GridStatusPub = rospy.Publisher('next_move', String, queue_size=10)
        
        rospy.init_node("game_engine")
        rospy.Subscriber('game_engine_cmd', String, self.game_engine_cmd_callback)
        self.GameEngineCmd = ''
        self.QuitCurrentSession = False
        self.ConvertIndex = [6, 3, 0, 7, 4, 1, 8, 5, 2] #[6, 3, 0, 7, 4, 1, 8, 5, 2]
        self.isThisRoundDone = False
        self.board = [' '] * 9
        self.player_name = ''
        self.player_marker = 'o'
        self.bot_name = 'baxter'
        self.bot_marker = 'x'
        self.winning_combos = (
        [6, 7, 8], [3, 4, 5], [0, 1, 2], [0, 3, 6], [1, 4, 7], [2, 5, 8],
        [0, 4, 8], [2, 4, 6],
        )
        self.corners = [0,2,6,8]
        self.sides = [1,3,5,7]
        self.middle = 4
        
        self.form = '''
           \t| %s | %s | %s |
           \t-------------
           \t| %s | %s | %s |
           \t-------------
           \t| %s | %s | %s |
           '''                    
    
    
    def print_board(self,board = None):
        "Display board on screen"
        if board is None:
            print self.form % tuple(self.board[6:9] + self.board[3:6] + self.board[0:3])
        else:
            # when the game starts, display numbers on all the grids
            print self.form % tuple(board[6:9] + board[3:6] + board[0:3])
                
    def game_engine_cmd_callback(self, msg):
        
        self.GameEngineCmd = msg.data
        print "Cmd Received: ", self.GameEngineCmd
        
    def get_game_engine_cmd(self):
        
        cur_cmd = self.GameEngineCmd
        
        while cur_cmd == '':
            
            cur_cmd = self.GameEngineCmd
            rospy.sleep(0.1)
            
        self.GameEngineCmd = ''
        
        return cur_cmd
    
    def interpret_game_engine_cmd(self, cmd_string):
        
        #game_status:b/h:x x x o o o x x x
        #game_engine:b/h:start, game_engine:b/h/quit_session
        print "Interpret Incoming Command..."
        cmd_segments = cmd_string.split(':')
        if len(cmd_segments) != 3:
            print "Incoming Command Segments not Correct"
            return '', '', []
        
        if cmd_segments[0] == 'game_status' or cmd_segments[0] == 'update_grid':
            
            new_list = cmd_segments[2].split()
            if len(new_list) != 9:
                print "Incoming game status grid number not right"
                return '', '', []
            new_board = [' '] *9
            counter = 0
            for item in new_list:
                
                if item == 'b':
                    new_board[counter] = ' '
                    #self.board[counter] = ' '
                else:
                    #self.board[counter] = item
                    new_board[counter] = item
                counter = counter + 1
                #elif item == 'cmd_segments[1]':
                    #new_board[new_list.index(item)] = 'b'
                #else:
                    #new_board[new_list.index(item)] = 'h'
            new_board1 = [' '] * 9
            new_board1[2] = new_board[0]
            new_board1[5] = new_board[1]
            new_board1[8] = new_board[2]
            new_board1[1] = new_board[3]
            new_board1[4] = new_board[4]
            new_board1[7] = new_board[5]
            new_board1[0] = new_board[6]
            new_board1[3] = new_board[7]
            new_board1[6] = new_board[8]
            
            
            print "Incoming New Board..."
            self.print_board(new_board1)
                    
            return cmd_segments[0], cmd_segments[1],new_board1
        elif cmd_segments[0] == 'game_engine':
            
            print "received game_engine command..."
            return 'game_engine', cmd_segments[1], cmd_segments[2]
        
        
        return '', '', []
    
    def is_winner(self, board, marker):
        #print "check if this marker will win the game"
        
        for combo in self.winning_combos:
            if (board[combo[0]] == board[combo[1]] == board[combo[2]] == marker):
                return True
        return False
    
    def get_bot_move(self, marker1, marker2):
        
        # check if bot can win in the next move
        for i in range(0,len(self.board)):
            board_copy = copy.deepcopy(self.board)
            if self.is_space_free(board_copy, i):
                self.make_move(board_copy,i,marker1)
                if self.is_winner(board_copy, marker1):
                    return i
        
              
        # check if player could win on his next move
        for i in range(0,len(self.board)):
            board_copy = copy.deepcopy(self.board)
            if self.is_space_free(board_copy, i):
                self.make_move(board_copy,i, marker2)
                if self.is_winner(board_copy, marker2):
                    return i
        

        # check for space in the corners, and take it
        move = self.choose_random_move(self.corners)
        if move != None:
            return move

        # If the middle is free, take it
        if self.is_space_free(self.board,self.middle):
            return self.middle
                

        # else, take one free space on the sides
        return self.choose_random_move(self.sides)
    
    def is_space_free(self, board, index):
        "checks for free space of the board"
        # print "SPACE %s is taken" % index
        return board[index] == ' '
    
    
    def is_board_full(self):
        "checks if the board is full"
        for i in range(1,9):
            if self.is_space_free(self.board, i):
                return False
        return True
    
    def make_move(self,board,index,move):
        board[index] =  move

    def choose_random_move(self, move_list):
        possible_winning_moves = []
        for index in move_list:
            if self.is_space_free(self.board, index):
                possible_winning_moves.append(index)
                if len(possible_winning_moves) != 0:
                    return random.choice(possible_winning_moves)
                else:
                    return None
                
    
    
    def get_player_move(self):
        move = int(input("Pick a spot to move: (1-9) "))
        while move not in [1,2,3,4,5,6,7,8,9] or not self.is_space_free(self.board,move-1) :
            move = int(input("Invalid move. Please try again: (1-9) "))
        return move - 1
    
    def find_new_move(self, new_list):
        
        counter = 0 
        for item in self.board:
            #print "item: ", item
            if item != new_list[counter]:
                #print "Find it", new_list[counter]
                return counter
            counter = counter + 1
        
        return -1
                
        
    
    def enter_game_loop(self,turn, new_id):
        "starts the main game loop"
        #self.QuitCurrentSession = False
        player = turn #h for human, b for bot
        print "Player is: ", player
        print "IsRound: ", self.isThisRoundDone
        print "IsSession: ", self.QuitCurrentSession
        #isThisRound = True
        msg_string = ''
        print "New Id: ", new_id
        while (not self.isThisRoundDone) and (not self.QuitCurrentSession):
            print "Ready to get a result..."
            if player == 'x':
                #user_input = self.get_player_move()
                self.make_move(self.board,new_id, 'x')
                self.print_board()
                
                if self.is_winner(self.board, 'x') or self.is_winner(self.board, 'o'):
                    print "x won..."
                    msg_string = 'win 0'
                    self.reset_game()
                    self.QuitCurrentSession = True
                    #break
                elif self.is_board_full():
                    self.print_board()
                    print "\n\t-- Match Draw --\t\n"
                    msg_string = 'draw 0'
                    self.reset_game()
                    self.QuitCurrentSession = True
                    #break
                else:
                    player_move =  self.get_bot_move('o', 'x')
                    #self.make_move(self.board,player_move, 'o')
                    msg_string = 'o' + ' ' + str(self.ConvertIndex[player_move])
                    
                    #print "new move o to : ", self.ConvertIndex[player_move]
                    self.print_board()
                    
                if msg_string!=' ' or msg_string!='':
                    print "Send Message back to game controller: ", msg_string    
                    self.GridStatusPub.publish(msg_string)
                else:
                    print "Msg string is space or empty..."
                
                self.isThisRoundDone = True
            
            elif player == 'o':
                
                self.make_move(self.board,new_id, 'o')
                self.print_board()
                
                if self.is_winner(self.board, 'x') or self.is_winner(self.board, 'o'):
                    print "o won..."
                    msg_string = 'win 1'
                    self.reset_game()
                    self.QuitCurrentSession = True
                elif self.is_board_full():
                    self.print_board()
                    print "\n\t-- Match Draw --\t\n"
                    msg_string = 'draw 0'
                    self.reset_game()
                    self.QuitCurrentSession = True
                else:
                    player_move =  self.get_bot_move('x', 'o')
                    #self.make_move(self.board,player_move, 'x')
                    msg_string = 'x' + ' ' + str(self.ConvertIndex[player_move])
                    print "new move x to : ", self.ConvertIndex[player_move]
                    
                    #self.print_board()
                
                
                
                
                if msg_string!=' ' or msg_string!='':  
                    print "Send Message back to game controller: ", msg_string  
                    self.GridStatusPub.publish(msg_string)
                else:
                    print "Msg string is space or empty..."
    
                
                self.isThisRoundDone = True
            else:
                print "Error, Player Marker not 'o' or 'x'..."
        
        print "Quit game loop..."
    


    def reset_game(self):
        
        self.isThisRoundDone = False
        self.board = [' '] * 9
        self.player_name = ''
        self.player_marker = 'o'
        self.bot_name = 'baxter'
        self.bot_marker = 'x'
        self.QuitCurrentSession = False
        
    def run(self):
        
        while not rospy.is_shutdown():
            
            print "Wait for Input"
            cmd = self.get_game_engine_cmd()
            
            action, turn, target_list = self.interpret_game_engine_cmd(cmd)
            print action, turn, target_list
            
            if action == 'game_engine' and target_list=='quit_session':
                
                self.QuitCurrentSession = True
                print "Game Session Quits..."
                self.reset_game()
            
            elif action == 'game_engine' and target_list=='start':
                
                self.QuitCurrentSession = False
                print "Game Session Starts..."
                self.reset_game()
                self.isThisRoundDone = False
                print "this round at start: ", self.isThisRoundDone
                print "this session at start: ", self.QuitCurrentSession
                print "Board at start: ", self.board
                
                
                
            elif action == 'game_status':
                
                self.isThisRoundDone = False
                print "Grid Status: ", self.board
                print "New Grid Status: ", target_list
                new_id = self.find_new_move(target_list)
                self.board = list(target_list)
                print "New Id: ", new_id
                
                #if turn == 'o':
                self.enter_game_loop(turn, new_id)
                #elif turn == 'x':
                    #self.enter_game_loop('o', new_id)
            elif action == 'update_grid':
                
                # To Do, Add Draw 
                
                print "Just update grid status..."
                self.board = list(target_list)
                msg_string = ''
                if self.is_winner(self.board, 'x'):
                    print "x won..."
                    msg_string = 'win 0'
                    self.reset_game()
                    self.QuitCurrentSession = False

                    
                elif self.is_winner(self.board, 'o'):
                    
                    print "o won..."
                    msg_string = 'win 1'
                    self.reset_game()
                    self.QuitCurrentSession = False
                    
                elif self.is_board_full():
                    
                    print "It's a draw"
                    msg_string = 'draw 0'
                    self.reset_game()
                    self.QuitCurrentSession = False
                    
                else:
                    print "No winner yet..."
                    msg_string = 'nowin 2'
                    
                if msg_string != '' or msg_string != '':
                    
                    self.GridStatusPub.publish(msg_string)
                    
                
                pass
            
            
            rospy.sleep(0.1)
            


def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)
   
def main():

    signal.signal(signal.SIGINT, signal_handler)
    game = GameEngine()
    
    game.run()
    

if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()