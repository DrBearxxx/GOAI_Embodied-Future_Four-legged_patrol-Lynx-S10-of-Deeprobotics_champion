"""Reproducible local debug APK build using installed Android build tools, no Gradle."""
from pathlib import Path
import os, subprocess, zipfile, hashlib, json, argparse, shutil
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--unsigned',action='store_true')
parser.add_argument('--keystore',type=Path)
args=parser.parse_args()
ROOT=Path(__file__).resolve().parent
JAVA=Path('C:/Program Files/Eclipse Adoptium/jdk-17.0.20.8-hotspot/bin')
SDK=Path('C:/Users/ming/AppData/Local/Android/Sdk')
BT=SDK/'build-tools/36.0.0'
ANDROID=SDK/'platforms/android-36/android.jar'
BUILD=ROOT/'build'
for name in ('classes','dex','gen','sdk'): (BUILD/name).mkdir(parents=True,exist_ok=True)
aar=ROOT/'vendor/rcsdk-v1.9.2.aar'
with zipfile.ZipFile(aar) as z:
    for name in z.namelist():
        if name=='classes.jar' or name.startswith(('res/','assets/','jni/')): z.extract(name,BUILD/'sdk')
kotlin=Path('C:/Users/ming/.gradle/caches/modules-2/files-2.1/org.jetbrains.kotlin/kotlin-stdlib/2.2.10/30de6faa127a4a012db8e71bf1b9c0a99b1402b2/kotlin-stdlib-2.2.10.jar')
def run(args):
    subprocess.run([str(x) for x in args],check=True,env=dict(os.environ,JAVA_HOME=str(JAVA.parent)))
run([BT/'aapt.exe','package','-f','-M',ROOT/'android/AndroidManifest.xml','-S',BUILD/'sdk/res','-A',BUILD/'sdk/assets','-A',ROOT/'android/assets','-I',ANDROID,'-F',BUILD/'resources.apk','-J',BUILD/'gen','--custom-package','com.skydroid.rcsdk'])
sources=list((ROOT/'android/src').rglob('*.java'))+list((BUILD/'gen').rglob('*.java'))
run([JAVA/'javac.exe','-encoding','UTF-8','-source','8','-target','8','-classpath',os.pathsep.join(map(str,[ANDROID,BUILD/'sdk/classes.jar',kotlin])),'-d',BUILD/'classes',*sources])
run([JAVA/'jar.exe','cf',BUILD/'app.jar','-C',BUILD/'classes','.'])
run([BT/'d8.bat','--lib',ANDROID,'--min-api','26','--output',BUILD/'dex',BUILD/'app.jar',BUILD/'sdk/classes.jar',kotlin])
with zipfile.ZipFile(BUILD/'resources.apk') as src,zipfile.ZipFile(BUILD/'unsigned.apk','w',zipfile.ZIP_DEFLATED) as dest:
    for item in src.infolist(): dest.writestr(item,src.read(item.filename))
    for path in (BUILD/'dex').glob('*.dex'): dest.write(path,path.name)
    for path in (BUILD/'sdk/jni/arm64-v8a').glob('*.so'): dest.write(path,'lib/arm64-v8a/'+path.name)
run([BT/'zipalign.exe','-f','4',BUILD/'unsigned.apk',BUILD/'aligned.apk'])
key=args.keystore or BUILD/'development.keystore'
apk=BUILD/('GOAI-Navigation-unsigned.apk' if args.unsigned else 'GOAI-Navigation.apk')
if args.unsigned:
    shutil.copy2(BUILD/'aligned.apk',apk)
else:
    if not key.is_file():
        raise SystemExit('Original signing key is unavailable. Use --keystore PATH, or --unsigned to compile without replacing the application identity.')
    run([BT/'apksigner.bat','sign','--ks',key,'--ks-pass','pass:android','--key-pass','pass:android','--out',apk,BUILD/'aligned.apk'])
    run([BT/'apksigner.bat','verify',apk])
print(json.dumps({'apk':str(apk),'signed':not args.unsigned,'sha256':hashlib.sha256(apk.read_bytes()).hexdigest(),'sdk_sha256':hashlib.sha256(aar.read_bytes()).hexdigest()},ensure_ascii=False))
