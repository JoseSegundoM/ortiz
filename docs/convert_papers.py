import os
from markitdown import MarkItDown

# Initialize the MarkItDown converter
md = MarkItDown()

# Use the directory where this script is located
papers_dir = os.path.dirname(os.path.abspath(__file__))

print(f"Scanning directory '{papers_dir}' for files to convert...")

for filename in os.listdir(papers_dir):
    # Skip python scripts, markdown files, and hidden files
    if filename.endswith(".py") or filename.endswith(".md") or filename.startswith("."):
        continue

    file_path = os.path.join(papers_dir, filename)
    
    # Only process files, not directories
    if not os.path.isfile(file_path):
        continue

    md_path = os.path.join(papers_dir, os.path.splitext(filename)[0] + ".md")
    
    print(f"Converting: {filename}...")
    try:
        result = md.convert(file_path)
        with open(md_path, "w", encoding="utf-8") as f:
            f.write(result.text_content)
        print(f"  -> Saved as {md_path}")
    except Exception as e:
        print(f"  -> Error converting {filename}: {e}")
