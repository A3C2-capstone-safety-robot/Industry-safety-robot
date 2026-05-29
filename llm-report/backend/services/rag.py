# backend/services/rag.py

import os
from pathlib import Path
from pypdf import PdfReader

BASE_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = BASE_DIR / "data"

documents = []


def split_text(text: str, chunk_size: int = 250, overlap: int = 100) -> list[str]:
    chunks = []

    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start += chunk_size - overlap

    return chunks


def load_pdfs():
    documents.clear()

    if not DATA_DIR.exists():
        print(f"[RAG] data folder not found: {DATA_DIR}")
        return

    for file_path in DATA_DIR.glob("*.pdf"):
        reader = PdfReader(str(file_path))
        text = ""

        for page in reader.pages:
            page_text = page.extract_text()
            if page_text:
                text += page_text + "\n"

        for chunk in split_text(text):
            documents.append({
                "source": file_path.name,
                "content": chunk,
            })

    print(f"[RAG] loaded {len(documents)} chunks from PDF files")


def calculate_score(query: str, content: str) -> int:
    query_words = set(query.lower().split())
    content_lower = content.lower()

    score = 0
    for word in query_words:
        if word in content_lower:
            score += 1

    return score


def retrieve_docs(query: str, top_k: int = 3) -> list[dict]:
    if not documents:
        load_pdfs()

    scored_docs = []

    for doc in documents:
        score = calculate_score(query, doc["content"])

        if score > 0:
            scored_docs.append({
                **doc,
                "score": score,
            })

    scored_docs.sort(key=lambda doc: doc["score"], reverse=True)

    return scored_docs[:top_k]


load_pdfs()