
import os
import uctypes
import uzlib
import machine

def rmdir(directory):
    os.chdir(directory)
    for f in os.listdir():
        try:
            os.remove(f)
        except OSError:
            pass
    for f in os.listdir():
        rmdir(f)
    os.chdir('..')
    os.rmdir(directory)

def copyfileobj(src, dest, length=512):
    if hasattr(src, "readinto"):
        buf = bytearray(length)
        while True:
            sz = src.readinto(buf)
            if not sz:
                break
            if sz == length:
                dest.write(buf)
            else:
                b = memoryview(buf)[:sz]
                dest.write(b)
    else:
        while True:
            buf = src.read(length)
            if not buf:
                break
            dest.write(buf)

# http://www.gnu.org/software/tar/manual/html_node/Standard.html
TAR_HEADER = {
    "name": (uctypes.ARRAY | 0, uctypes.UINT8 | 100),
    "size": (uctypes.ARRAY | 124, uctypes.UINT8 | 11),
}

DIRTYPE = "dir"
REGTYPE = "file"

def roundup(val, align):
    return (val + align - 1) & ~(align - 1)

class FileSection:

    def __init__(self, f, content_len, aligned_len):
        self.f = f
        self.content_len = content_len
        self.align = aligned_len - content_len

    def read(self, sz=65536):
        if self.content_len == 0:
            return b""
        if sz > self.content_len:
            sz = self.content_len
        data = self.f.read(sz)
        sz = len(data)
        self.content_len -= sz
        return data

    def readinto(self, buf):
        if self.content_len == 0:
            return 0
        if len(buf) > self.content_len:
            buf = memoryview(buf)[:self.content_len]
        sz = self.f.readinto(buf)
        self.content_len -= sz
        return sz

    def skip(self):
        sz = self.content_len + self.align
        if sz:
            buf = bytearray(16)
            while sz:
                s = min(sz, 16)
                self.f.readinto(buf, s)
                sz -= s

class TarInfo:

    def __str__(self):
        return "TarInfo(%r, %s, %d)" % (self.name, self.type, self.size)

class TarFile:

    def __init__(self, name=None, fileobj=None):
        if fileobj:
            self.f = fileobj
        else:
            self.f = open(name, "rb")
        self.subf = None

    def next(self):
            if self.subf:
                self.subf.skip()
            buf = self.f.read(512)
            if not buf:
                return None

            h = uctypes.struct(uctypes.addressof(buf), TAR_HEADER, uctypes.LITTLE_ENDIAN)

            # Empty block means end of archive
            if h.name[0] == 0:
                return None

            d = TarInfo()
            d.name = str(h.name, "utf-8").rstrip("\0")
            d.size = int(bytes(h.size), 8)
            d.type = [REGTYPE, DIRTYPE][d.name[-1] == "/"]
            self.subf = d.subf = FileSection(self.f, d.size, roundup(d.size, 512))
            return d

    def __iter__(self):
        return self

    def __next__(self):
        v = self.next()
        if v is None:
            raise StopIteration
        return v

    def extractfile(self, tarinfo):
        return tarinfo.subf

def untar_file(tar_fn):
    t = TarFile(tar_fn)
    for i in t:
        print(i)
        if i.type == DIRTYPE:
            print("Mkdir: " + i.name)
            try:
                os.mkdir(i.name.strip("/"))
            except:
                pass
        else:
            f = t.extractfile(i)
            print("Extract: " + i.name)
            fp = open(i.name, "wb")
            copyfileobj(f, fp)
            fp.close()

def unzip_file(zip_file, to_f):
    f = open(zip_file, 'rb')
    compressed_data = f.read()
    f.close()

    import gc
    gc.collect()
    #print(gc.mem_free())
    gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
    #print(gc.mem_free())
    decompressed_data = uzlib.decompress(compressed_data)
    f = open(to_f, "wb")
    f.write(decompressed_data)
    f.close()

def unzip_untar(hbz_file):
    untar_file(hbz_file)
    
    os.remove(hbz_file)

    for uhb_file in ["uhb01", "uhb02"]:
        zfn = uhb_file + ".zlib"
        tfn = uhb_file + ".tar"
        unzip_file(zfn, tfn)
        untar_file(tfn)
        os.remove(tfn)
    try:
        os.remove("@PaxHeader")
    except:
        pass
