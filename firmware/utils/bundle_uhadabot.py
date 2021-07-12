import zlib
import tarfile
import os.path
import os
from shutil import copyfile

def zip_file(in_file, to_f):
    f = open(in_file, "rb")
    td = f.read()
    f.close()

    cd = zlib.compress(td)
    
    f = open(to_f, "wb")
    f.write(cd)
    f.close()


def make_tarfile(output_filename, source_dir_list):
    with tarfile.open(output_filename, "w") as tar:
        for source_dir in source_dir_list:
            try:
                tar.add(source_dir, arcname=source_dir) 
            except:
                print(f"Could not find/bundle '{source_dir}'")


if __name__ == "__main__":
    make_tarfile("uhadabot01.tar", ["uhadabot/boot.py",
                                    "uhadabot/main.py",
                                    "uhadabot/ssd1306.py",
                                    "uhadabot/webrepl_cfg.py"])
    make_tarfile("uhadabot02.tar", ["uhadabot/uroslibpy"])
    make_tarfile("uhadabot03.tar", ["lib"])

    tarfile_list = []
    zlibfile_list = []
    for i in range(3):
        idx = i+1
        tar_fn = f"uhadabot0{idx}.tar"
        zlib_fn = f"uhb0{idx}.zlib"
        zip_file(tar_fn, zlib_fn)
        tarfile_list.append(tar_fn)
        zlibfile_list.append(zlib_fn)

    make_tarfile("uhadabot.hbz", zlibfile_list)
    
    for fn in tarfile_list:
        os.remove(fn)
    for fn in zlibfile_list:
        os.remove(fn)
