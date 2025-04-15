import os
from langchain_community.vectorstores import Chroma
from langchain.chains import RetrievalQA
from langchain.text_splitter import CharacterTextSplitter
from langchain_openai import OpenAIEmbeddings, ChatOpenAI
import config

class ChatGPT:
    def __init__(self):
        langs = ["ja", "en", "zh"]
        self.qa_chain = dict()

        for lang in langs:
            # 检查文本文件
            if not os.path.exists(config.TEXT_FILE_PATH[lang]):
                raise FileNotFoundError(f"テキストファイル '{config.TEXT_FILE_PATH[lang]}' が見つかりません。")

            # 读取文本
            with open(config.TEXT_FILE_PATH[lang], "r", encoding="utf-8") as f:
                text = f.read().strip()

            # 文本切分
            text_splitter = CharacterTextSplitter(
                separator=config.TEXT_FILE_SEPERATOR[lang],
                chunk_size=1500,
                chunk_overlap=50
            )
            docs = text_splitter.split_text(text)
            # 生成向量库
            embeddings = OpenAIEmbeddings(openai_api_key=config.API_KEY)
            db = Chroma.from_texts(docs, embeddings)
            retriever = db.as_retriever()

            # 构建 RAG 问答链
            self.qa_chain[lang] = RetrievalQA.from_chain_type(
                llm=ChatOpenAI(
                    model_name="gpt-4o-mini",
                    temperature=0,
                    openai_api_key=config.API_KEY
                ),
                chain_type="stuff",
                retriever=retriever
            )

    def get_response(self, prompt: str, lang) -> str:
        answer = self.qa_chain[lang].invoke(prompt)
        print("🤖 ChatGPT:", answer)
        return answer
