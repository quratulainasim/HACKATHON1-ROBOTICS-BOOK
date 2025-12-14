import os
import glob
from typing import List, Dict
from pathlib import Path
from datetime import datetime

class ContentReader:
    """
    Utility class to read content from docs/ folder
    Based on the requirement to extract content from markdown files in docs/ folder
    """

    def __init__(self, docs_path: str = "E:/my-AI-Robotics-Book/docs"):
        self.docs_path = Path(docs_path)

    def read_all_content(self) -> List[Dict]:
        """
        Read all markdown content from the docs/ folder
        Returns a list of content blocks with path, title, and content
        """
        content_blocks = []

        # Find all markdown files in the docs directory and subdirectories
        md_files = glob.glob(str(self.docs_path / "**/*.md"), recursive=True)
        md_files.extend(glob.glob(str(self.docs_path / "**/*.mdx"), recursive=True))

        for file_path in md_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract title from the file (first heading or filename)
                title = self._extract_title(content, file_path)

                # Get file modification time
                mod_time = datetime.fromtimestamp(os.path.getmtime(file_path))

                content_blocks.append({
                    'path': str(Path(file_path).relative_to(self.docs_path)),
                    'title': title,
                    'content': content,
                    'last_modified': mod_time.isoformat()
                })
            except Exception as e:
                print(f"Error reading file {file_path}: {str(e)}")
                continue

        return content_blocks

    def read_content_by_path(self, relative_path: str) -> Dict:
        """
        Read content from a specific path relative to docs/ folder
        """
        full_path = self.docs_path / relative_path

        if not full_path.exists():
            raise FileNotFoundError(f"File not found: {full_path}")

        with open(full_path, 'r', encoding='utf-8') as f:
            content = f.read()

        title = self._extract_title(content, str(full_path))
        mod_time = datetime.fromtimestamp(os.path.getmtime(full_path))

        return {
            'path': relative_path,
            'title': title,
            'content': content,
            'last_modified': mod_time.isoformat()
        }

    def get_available_files(self) -> List[str]:
        """
        Get list of all available markdown files in docs/ folder
        """
        md_files = glob.glob(str(self.docs_path / "**/*.md"), recursive=True)
        mdx_files = glob.glob(str(self.docs_path / "**/*.mdx"), recursive=True)

        all_files = md_files + mdx_files
        return [str(Path(f).relative_to(self.docs_path)) for f in all_files]

    def _extract_title(self, content: str, file_path: str) -> str:
        """
        Extract title from content (first heading) or use filename if no heading found
        """
        lines = content.split('\n')

        # Look for first heading (## or #)
        for line in lines:
            line = line.strip()
            if line.startswith('# '):
                return line[2:].strip()  # Remove '# ' prefix
            elif line.startswith('## '):
                return line[3:].strip()  # Remove '## ' prefix

        # If no heading found, use the filename
        return Path(file_path).stem.replace('-', ' ').replace('_', ' ').title()