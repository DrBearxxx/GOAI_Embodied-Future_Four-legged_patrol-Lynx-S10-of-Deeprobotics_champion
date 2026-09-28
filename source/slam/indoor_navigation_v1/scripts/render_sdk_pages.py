from pathlib import Path
import pypdfium2 as pdfium
root=Path(__file__).resolve().parents[1]
doc=pdfium.PdfDocument('E:/goai/goai_robotics_workspace/docs/source_and_operation/S10软件开发指南202607.pdf')
for page in (13,50,51,52,53):
    doc[page-1].render(scale=1.5).to_pil().save(root/f'tmp/pdfs/sdk-{page}.png')
