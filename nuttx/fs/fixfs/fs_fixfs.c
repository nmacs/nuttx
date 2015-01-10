/****************************************************************************
 * fs/fixfs/fs_fixfs.c
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/fixfs.h>
#include <nuttx/fs/dirent.h>

#include <arch/irq.h>

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_FIXFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * External Definitons
 ****************************************************************************/

extern int bchlib_setup(const char *blkdev, bool readonly, FAR void **handle);
extern int bchlib_teardown(FAR void *handle);
extern ssize_t bchlib_read(FAR void *handle, FAR char *buffer, size_t offset, size_t len);
extern ssize_t bchlib_write(FAR void *handle, FAR const char *buffer, size_t offset, size_t len);

extern const struct fixfs_file fixfs_files[];

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fixfs_file_s
{
  const struct fixfs_file *f;
  size_t pos;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File system methods */

static int     fixfs_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     fixfs_close(FAR struct file *filep);
static ssize_t fixfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t fixfs_write(FAR struct file *filep, const char *buffer,
                 size_t buflen);
static off_t   fixfs_seek(FAR struct file *filep, off_t offset, int whence);
static int     fixfs_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
static int     fixfs_sync(FAR struct file *filep);


static int     fixfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     fixfs_opendir(FAR struct inode *mountpt, const char *relpath,
                 FAR struct fs_dirent_s *dir);
static int     fixfs_readdir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);
static int     fixfs_rewinddir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);

static int     fixfs_bind(FAR struct inode *blkdriver,
                 FAR const void *data, FAR void **handle);
static int     fixfs_unbind(FAR void *handle, FAR struct inode **blkdriver);
static int     fixfs_statfs(FAR struct inode *mountpt,
                 FAR struct statfs *buf);

static int     fixfs_stat(FAR struct inode *mountpt,
                 FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations fixfs_operations =
{
  fixfs_open,       /* open */
  fixfs_close,      /* close */
  fixfs_read,       /* read */
  fixfs_write,      /* write */
  fixfs_seek,       /* seek */
  fixfs_ioctl,      /* ioctl */

  fixfs_sync,       /* sync */
  fixfs_dup,        /* dup */

  fixfs_opendir,    /* opendir */
  NULL,             /* closedir */
  fixfs_readdir,    /* readdir */
  fixfs_rewinddir,  /* rewinddir */

  fixfs_bind,       /* bind */
  fixfs_unbind,     /* unbind */
  fixfs_statfs,     /* statfs */

  NULL,             /* unlink */
  NULL,             /* mkdir */
  NULL,             /* rmdir */
  NULL,             /* rename */
  fixfs_stat        /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fixfs_open
 ****************************************************************************/

static int fixfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  const struct fixfs_file *f = fixfs_files;

  fvdbg("Open '%s'\n", relpath);

  while (f->name)
    {
      if (strcmp(f->name, relpath) == 0)
        {
          filep->f_priv = (void*)f;
          filep->f_pos = 0;
          return OK;
        }
      f++;
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: fixfs_close
 ****************************************************************************/

static int fixfs_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: fixfs_read
 ****************************************************************************/

static ssize_t fixfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  struct inode *inode;
  void *handler;
  const FAR struct fixfs_file *f;
  ssize_t ret;

  f       = filep->f_priv;
  DEBUGASSERT(f);
  inode   = filep->f_inode;
  DEBUGASSERT(inode);
  handler = inode->i_private;
  DEBUGASSERT(handler);

  fvdbg("buffer=%p buflen=%d file(size:%d, offset:%d pos:%d)\n", buffer, (int)buflen, (int)f->size, (int)f->offset, (int)filep->f_pos);

  if (filep->f_pos + buflen > f->size)
    {
      buflen = f->size - filep->f_pos;
    }

  if (buflen == 0)
    {
      return 0;
    }

  /* Call the handler's read routine */
  ret = bchlib_read(handler, buffer, f->offset + filep->f_pos, buflen);
  if (ret < 0)
    {
      return ret;
    }
  filep->f_pos += ret;

  return ret;
}

/****************************************************************************
 * Name: fixfs_write
 ****************************************************************************/

static ssize_t fixfs_write(FAR struct file *filep, const char *buffer,
                           size_t buflen)
{
  struct inode *inode;
  void *handler;
  const FAR struct fixfs_file *f;
  ssize_t ret;

  f       = filep->f_priv;
  DEBUGASSERT(f);
  inode   = filep->f_inode;
  DEBUGASSERT(inode);
  handler = inode->i_private;
  DEBUGASSERT(handler);

  lldbg("buffer=%p buflen=%d file(size:%d, offset:%d pos:%d)\n", buffer, (int)buflen, (int)f->size, (int)f->offset, (int)filep->f_pos);

  if (filep->f_pos + buflen > f->size)
    {
      buflen = f->size - filep->f_pos;
    }

  if (buflen == 0)
    {
      return 0;
    }

  /* Call the handler's read routine */
  ret = bchlib_write(handler, buffer, f->offset + filep->f_pos, buflen);
	lldbg("ret: %i\n", ret);
  if (ret < 0)
    {
      return ret;
    }
  filep->f_pos += ret;

  return ret;
}

/****************************************************************************
 * Name: fat_seek
 ****************************************************************************/

static off_t fixfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  struct fixfs_file *priv;
  off_t position;

  /* Sanity checks */

  priv    = filep->f_priv;
  DEBUGASSERT(priv);

  /* Map the offset according to the whence option */

  switch (whence)
    {
      case SEEK_SET: /* The offset is set to offset bytes. */
          position = offset;
          break;

      case SEEK_CUR: /* The offset is set to its current location plus
                      * offset bytes. */

          position = offset + filep->f_pos;
          break;

      case SEEK_END: /* The offset is set to the size of the file plus
                      * offset bytes. */

          position = offset + priv->size;
          break;

      default:
          return -EINVAL;
    }

  if (position < 0 || position > priv->size)
    {
      return -EINVAL;
    }

  filep->f_pos = position;

  return position;
}

/****************************************************************************
 * Name: fixfs_ioctl
 ****************************************************************************/

static int fixfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  fvdbg("cmd: %d arg: %08lx\n", cmd, arg);

  /* No IOCTL commands supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: fixfs_sync
 ****************************************************************************/

static int fixfs_sync(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: fixfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int fixfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  newp->f_priv = oldp->f_priv;

  return OK;
}

/****************************************************************************
 * Name: fixfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int fixfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s *dir)
{
  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  if (relpath != 0 && relpath[0] != '\0')
    {
      return -ENOENT;
    }

  /* Set the index to the first entry */

  dir->u.fixfs.fb_index = 0;

  return OK;
}

/****************************************************************************
 * Name: fixfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int fixfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  FAR const char *name;
  unsigned int index;
  int ret;

  DEBUGASSERT(dir);

  /* Have we reached the end of the directory */

  index = dir->u.fixfs.fb_index;
  name = fixfs_files[index].name;
  if (name == NULL)
    {
      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      fvdbg("Entry %d: End of directory\n", index);
      ret = -ENOENT;
    }
  else
    {
      /* Save the filename and file type */

      fvdbg("Entry %d: \"%s\"\n", index, name);
      dir->fd_dir.d_type = DTYPE_FILE;
      strncpy(dir->fd_dir.d_name, name, NAME_MAX+1);

      /* The application list is terminated by an entry with a NULL name.
       * Therefore, there is at least one more entry in the list.
       */

      index++;

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private fb_index.
       */

      dir->u.fixfs.fb_index = index;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: fixfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int fixfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  dir->u.fixfs.fb_index = 0;
  return OK;
}

/****************************************************************************
 * Name: fixfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int fixfs_bind(FAR struct inode *blkdriver, const void *data,
                       void **handle)
{
  char blkdev[32];

  snprintf(blkdev, sizeof(blkdev), "/dev/%s", blkdriver->i_name);
  fvdbg("blkdriver:%s\n", blkdev);

  return bchlib_setup(blkdev, 0, handle);
}

/****************************************************************************
 * Name: fixfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int fixfs_unbind(void *handle, FAR struct inode **blkdriver)
{
  return bchlib_teardown(handle);
}

/****************************************************************************
 * Name: fixfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int fixfs_statfs(struct inode *mountpt, struct statfs *buf)
{
  /* Fill in the statfs info */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = FIXFS_MAGIC;
  buf->f_bsize   = 0;
  buf->f_blocks  = 0;
  buf->f_bfree   = 0;
  buf->f_bavail  = 0;
  buf->f_namelen = NAME_MAX;
  return OK;
}

/****************************************************************************
 * Name: fixfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int fixfs_stat(struct inode *mountpt, const char *relpath,
                       struct stat *buf)
{
  int ret = -ENOENT;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "");

  if (relpath == NULL || relpath[0] == '\0')
    {
      buf->st_mode = S_IFDIR|S_IROTH|S_IRGRP|S_IRUSR;
      ret = OK;
    }
  else
    {
      const struct fixfs_file *f = fixfs_files;
      while (f->name)
        {
          if (strcmp(f->name, relpath) == 0)
            {
              buf->st_mode = S_IFREG|S_IROTH|S_IRGRP|S_IRUSR|S_IWOTH|S_IWGRP|S_IWUSR;
              buf->st_size = f->size;
              ret = OK;
              break;
            }
          f++;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_FIXFS */
