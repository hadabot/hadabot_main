import zlib

def main():
    fn = "uhadabot.hbz"
    fp = open(fn, "rb")

    ddd = fp.read()

    fp.close()

    #d = zlib.decompressobj(-15)
    #ddata = d.decompress(ddd)
    #ddata = d.flush()
    ddata = zlib.decompress(ddd)


    tp = open("tmp.tar", "wb")
    tp.write(ddata)
    tp.close()

if __name__ == "__main__":
    main()
