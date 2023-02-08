"""
ECE487: Tic-Tac-Toe
"""

import numpy as np


"""
If board_size = 4, the board will be
array([[0, 0, 0, 0],
       [0, 0, 0, 0],
       [0, 0, 0, 0],
       [0, 0, 0, 0]])
"""

board_size = 5
board = np.zeros([board_size, board_size], dtype="int")


def display_board():
    """
    Display the current game board
    :return: None
    """
    print("_______")
    for row in board:
        print(row)


def update_board(player, row, col):
    """
    Update the current board. If the row and column index is already played,
    it will not update the board and return False.
    :param player: Player ID. It must be 1 or 2
    :param row: the row index of the game board
    :param col: the column index of the game board
    :return: True if the game board is successfully updated.
    and False if there are errors.
    """
    if player < 1 or player > 2:
        print("Error: player must be either 1 or 2.")
        return False

    if row < 0 or row > board_size or col < 0 or col > board_size:
        print("Error: invalid row or column index.")
        return False

    if board[row][col] != 0:
        print(f"Error: Row:{row} Col:{col} has already been played.")
        return False

    # updated the current game board.
    board[row][col] = player
    return True


def check_game():
    """
    Print the winner of the game and return True if the game is over.
    Return False if the game is not over yet
    :return: True if the game is over and False if not.
    """

    # Check if the game is won horizontally
    # Print "Player X wins horizontally" if player X wins the game.
    for row in board:
        if row[0] != 0 and np.all(row == row[0]):
            print(f"Player {row[0]} wins horizontally")
            return True

    # Check if the game is won vertically.
    # Print "Player X wins vertically" if player X wins the game.
    # Write your code here
    x = np.transpose(board)
    for col in x:
        if col[0] != 0 and np.all(col == col[0]):
            print(f"Player {col[0]} wins vertically")
            return True

    # Check if the game is won diagonally
    # Print "Player X wins diagonally" if player X wins the game.
    # Write your code here
    y = np.diagonal(board)
    if y[0] != 0 and np.all(y == y[0]):
        print(f"Player {y[0]} wins diagonally")
        return True

    # Check if the game is won anti-diagonally
    # Print "Player X wins anti-diagonally" if player X wins the game.
    # Write your code here
    z = np.fliplr(board).diagonal()
    if z[0] != 0 and np.all(z == z[0]):
        print(f"Player {z[0]} wins anti - diagonally")
        return True

    # Check if the game is a draw - no zero left in the board
    # Print "The game is a draw" if so.
    # Write your code here
    if np.all(row != 0) and np.all(col != 0):
        print(f"The game is a draw :(")
        return True

    return False


def main():

    players = [1, 2]
    current_player = 0

    while True:
        display_board()
        row = input(f"Player {players[current_player]}: Enter the row you want to play (0,...,{board_size-1}): ")
        col = input(f"Player {players[current_player]}: Enter the column you want to play (0,...,{board_size-1}): ")

        if not update_board(players[current_player], int(row), int(col)):
            print("Play again")
            continue

        if check_game():
            display_board()
            break

        current_player ^= 1


if __name__ == '__main__':
    main()

