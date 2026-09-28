from pathlib import Path
from pypdf import PdfReader
root=Path(__file__).resolve().parents[1]
out=root/'tmp/pdfs';out.mkdir(parents=True,exist_ok=True)
pdf=Path('E:/goai/goai_robotics_workspace/docs/source_and_operation/S10软件开发指南202607.pdf')
reader=PdfReader(pdf)
for i,page in enumerate(reader.pages):
    text=page.extract_text() or ''
    (out/f'page-{i+1:02d}.txt').write_text(text,encoding='utf-8')
    print(i+1,text[:120].replace('\n',' '))
