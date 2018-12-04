#!/usr/bin/env python

import curses

KEY_A = 97
KEY_D = 100
KEY_E = 101
KEY_S = 115
KEY_W = 119

def main():
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()

    throttle = 0
    x_rot = 0
    y_rot = 0
    i = 0
    while True:
        c = stdscr.getch()
        i += 1
        if c == KEY_E:
            break
        elif c == KEY_W:
            throttle += 1
        elif c == KEY_S:
            throttle -= 1
        elif c == KEY_A:
            x_rot -= 1
        elif c == KEY_D:
            x_rot += 1
        if i % 10 == 0:
            output = "{} {} {}".format(throttle, x_rot, y_rot)
            stdscr.addstr(0, 0, output)
            stdscr.clrtoeol()
            stdscr.refresh()

    curses.nocbreak()
    stdscr.keypad(False)
    curses.echo()

    curses.endwin()

if __name__ == "__main__":
    main()
