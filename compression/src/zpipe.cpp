#include "zpipe.h"

int Zpipe::def(FILE* source, FILE* dest, int level) {
    int ret, flush;
    unsigned have;
    z_stream strm;
    unsigned char in[CHUNK];
    unsigned char out[CHUNK];

    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    ret = deflateInit(&strm, level);
    if (ret != Z_OK)
        return ret;

    do {
        strm.avail_in = fread(in, 1, CHUNK, source);
        if (ferror(source)) {
            (void)deflateEnd(&strm);
            return Z_ERRNO;
        }
        flush = feof(source) ? Z_FINISH : Z_NO_FLUSH;
        strm.next_in = in;

        do {
            strm.avail_out = CHUNK;
            strm.next_out = out;
            ret = deflate(&strm, flush);    /* anyone error value */
            assert(ret != Z_STREAM_ERROR);
            have = CHUNK - strm.avail_out;
            if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
                (void)deflateEnd(&strm);
                return Z_ERRNO;
            }
        } while (strm.avail_out == 0);
        assert(strm.avail_in == 0);

    } while (flush != Z_FINISH);
    assert(ret == Z_STREAM_END);

    /* limpar e retornar */
    (void)deflateEnd(&strm);
    return Z_OK;
}

int Zpipe::inf(FILE* source, FILE* dest) {
    int ret;
    unsigned have;
    z_stream strm;
    unsigned char in[CHUNK];
    unsigned char out[CHUNK];

    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit(&strm);
    if (ret != Z_OK)
        return ret;

    do {
        strm.avail_in = fread(in, 1, CHUNK, source);
        if (ferror(source)) {
            (void)inflateEnd(&strm);
            return Z_ERRNO;
        }
        if (strm.avail_in == 0)
            break;
        strm.next_in = in;

        do {
            strm.avail_out = CHUNK;
            strm.next_out = out;
            ret = inflate(&strm, Z_NO_FLUSH);
            assert(ret != Z_STREAM_ERROR);
            switch (ret) {
            case Z_NEED_DICT:
                ret = Z_DATA_ERROR;
            case Z_DATA_ERROR:
            case Z_MEM_ERROR:
                (void)inflateEnd(&strm);
                return ret;
            }
            have = CHUNK - strm.avail_out;
            if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
                (void)inflateEnd(&strm);
                return Z_ERRNO;
            }
        } while (strm.avail_out == 0);

    } while (ret != Z_STREAM_END);

    (void)inflateEnd(&strm);
    return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

void Zpipe::zerr(int ret) {
    std::cerr << "zpipe: ";
    switch (ret) {
    case Z_ERRNO:
        if (ferror(stdin)) {
            std::cerr << "Error to read stdin . " << '\n';
        }
        else if (ferror(stdout)) {
            std::cerr << "Error to writing stdout." << '\n';
        }
        break;
    case Z_STREAM_ERROR:
        std::cerr << "Invalid compression level . " << '\n';
        break;
    case Z_DATA_ERROR:
        std::cerr << "Empty data, invalid or incomplete. " << '\n';
        break;
    case Z_MEM_ERROR:
        std::cerr << "No memory. " << '\n';
        break;
    case Z_VERSION_ERROR:
        std::cerr << "zlib version is incompatible. " << '\n';
    }
}

void Zpipe::usage(char** argv) {
    std::cout << "Compress: " << argv[0] << " file.txt # Generate: file.txt.Z " << '\n';
    std::cout << "Decompress: " << argv[0] << " -d file.txt.Z # Generate: file.txt " << '\n';
}