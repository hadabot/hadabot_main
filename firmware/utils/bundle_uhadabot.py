import zlib
import tarfile
import os.path
import os

def zip_file(in_file, to_f):
    f = open(in_file, "rb")
    td = f.read()
    f.close()

    cd = zlib.compress(td)
    f = open(to_f, "wb")
    f.write(cd)
    f.close()


def make_tarfile(output_filename, source_dir):
    with tarfile.open(output_filename, "w") as tar:
        tar.add(source_dir, arcname=os.path.basename(source_dir))

    hbz_fn = output_filename.split(".")[0]
    zip_file(output_filename, f"{hbz_fn}.hbz")
    os.remove(output_filename)


if __name__ == "__main__":
    make_tarfile("uhadabot.tar", "uhadabot")
