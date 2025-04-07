#!/usr/bin/env python3

import os
from langchain_community.vectorstores import Chroma
from langchain.chains import RetrievalQA
from langchain.text_splitter import CharacterTextSplitter
from langchain_openai import OpenAIEmbeddings
from langchain_openai import ChatOpenAI

# Set the OpenAI API Key
api_key = os.getenv("OPENAI_API_KEY")
if not api_key:
    raise ValueError("環境変数 'OPEN_AI_KEY' が設定されていません")

# Load text from file
text_file_path = "../resource/mikuni_essay.txt"

# Check if files exist
if not os.path.exists(text_file_path):
    raise FileNotFoundError(f"テキストファイル '{text_file_path}' が見つかりません。")

# Read text and query
with open(text_file_path, "r", encoding="utf-8") as f:
    text = f.read().strip()

# Split text
text_splitter = CharacterTextSplitter(
    separator = "。",
    chunk_size = 1500,
    chunk_overlap  = 50
)
docs = text_splitter.split_text(text)

# Instantiating an Embedding
embeddings = OpenAIEmbeddings(openai_api_key=api_key)

# Create a VectorStore
db = Chroma.from_texts(docs, embeddings)
retriever = db.as_retriever()

# Text generation using RAG
chat = ChatOpenAI(
    model_name = "gpt-4o-mini",  # or "gpt-3.5-turbo", "gpt-4o"
    temperature = 0,
    openai_api_key = api_key
)
qa_chain = RetrievalQA.from_chain_type(
    llm = chat,
    chain_type = "stuff",
    retriever = retriever
)

# chat loop
print("Chatbotを開始します。質問を入力してください。（終了するには 'exit' と入力）")
while True:
    query = input("質問： ").strip()
    if query.lower() == "exit":
        print("Chatbotを終了します。")
        break
    
    result = qa_chain.invoke(query)
    print("回答：", result)




