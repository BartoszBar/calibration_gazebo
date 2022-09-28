import argparse



def main():
    parser = argparse.ArgumentParser(description='Value to print')
    parser.add_argument('-r', '--r', type=float, metavar='', default=1, help='radious of cylinder')
    args = parser.parse_args()
    print(args.r)



if __name__ == "__main__":
    main()
