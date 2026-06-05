import PyPDF2

try:
    reader = PyPDF2.PdfReader('paper.pdf')
    text = '\n'.join([p.extract_text() for p in reader.pages])
    with open('paper_text.txt', 'w', encoding='utf-8') as f:
        f.write(text)
except Exception as e:
    print(f"Error: {e}")
