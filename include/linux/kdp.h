#ifndef __KDP_H__
#define __KDP_H__

#ifndef __ASSEMBLY__
#include <linux/mm_types.h>
#include <linux/stddef.h>
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/binfmts.h>
#include <linux/uh.h>

#define __kdp_ro __section(.kdp_ro)
#define __lsm_ro_after_init_kdp __section(.kdp_ro)

/* uH_RKP Command ID */
enum __KDP_CMD_ID {
	KDP_INIT = 0x40,
	NS_INIT = 0x41,
	CRED_INIT = 0x42,
	SET_CRED_PGD = 0x43,
	SET_FREEPTR = 0x44,
	SELINUX_CRED_FREE = 0x45,
	PREPARE_RO_CRED = 0x46,
	PGD_RWX = 0x48,
	CHECK_DMAP = 0x4A,
	MARK_PPT = 0x4B,
	SER_SP_RO = 0x4E,
	SET_NS_RO = 0x4F,
	SET_CRED_RO = 0x50,
	ALLOC_VFSMOUNT = 0x52,
	SET_NS_ROOT_SB = 0x53,
	SET_NS_FLAGS = 0x54,
	SET_NS_DATA = 0x55,
	SET_NS_SB_VFSMOUNT = 0x56,
	PROTECT_SELINUX_VAR = 0x60,
};

#define CRED_JAR_RO		"cred_jar_ro"
#define TSEC_JAR		"tsec_jar"
#define VFSMNT_JAR		"vfsmnt_cache"

struct kdp_init {
	u32 credSize;
	u32 sp_size;
	u32 pgd_mm;
	u32 uid_cred;
	u32 euid_cred;
	u32 gid_cred;
	u32 egid_cred;
	u32 bp_pgd_cred;
	u32 bp_task_cred;
	u32 type_cred;
	u32 security_cred;
	u32 usage_cred;
	u32 cred_task;
	u32 mm_task;
	u32 pid_task;
	u32 rp_task;
	u32 comm_task;
	u32 bp_cred_secptr;
	u32 task_threadinfo;
	u64 verifiedbootstate;
	struct {
		u64 empty;
		u64 ss_initialized_va;
	} selinux;
};

//kernel/cred.c
enum __CRED_CMD_ID {
	CMD_COPY_CREDS = 0,
	CMD_COMMIT_CREDS,
	CMD_OVRD_CREDS,
};

extern bool kdp_enable;
extern void __init kdp_init(void);
extern bool is_kdp_kmem_cache(struct kmem_cache *s);

#ifdef CONFIG_KDP_CRED
/***************** KDP_CRED *****************/
struct ro_rcu_head {
	/* RCU deletion */
	union {
		int non_rcu;		/* Can we skip RCU deletion? */
		struct rcu_head	rcu;	/* RCU deletion hook */
	};
	void *bp_cred;
	void *reflected_cred;
};

struct kdp_usecnt {
	atomic_t kdp_use_cnt;
	struct ro_rcu_head kdp_rcu_head;
};

struct cred_param {
	struct cred *cred;
	struct cred *cred_ro;
	void *use_cnt_ptr;
	void *sec_ptr;
	unsigned long type;
	union {
		void *task_ptr;
		u64 use_cnt;
	};
};

#define KDP_IS_NONROOT(x)	((x->cred->type) >> 1 & 1)
#define CHECK_ROOT_UID(x)	\
	(x->cred->uid.val == 0 || x->cred->gid.val == 0 || \
	 x->cred->euid.val == 0 || x->cred->egid.val == 0 || \
	 x->cred->suid.val == 0 || x->cred->sgid.val == 0)

#define GET_ROCRED_RCU(cred) 	((struct ro_rcu_head *)((atomic_t *)cred->use_cnt + 1))

/*
After KDP endbled, argument of override_creds will not become the current->cred.
But some code trys to put_creds the current->cred, to free the resource of cred
which was allocated before the override_creds. In those case, we need to find the
original cred by below function.
*/
#define GET_REFLECTED_CRED(cred) 	((struct cred *)GET_ROCRED_RCU(cred)->reflected_cred)

#define ROCRED_UC_READ(x)			atomic_read(x->use_cnt)
#define ROCRED_UC_INC(x)			atomic_inc(x->use_cnt)
#define ROCRED_UC_DEC_AND_TEST(x)	atomic_dec_and_test(x->use_cnt)
#define ROCRED_UC_INC_NOT_ZERO(x)	atomic_inc_not_zero(x->use_cnt)
#define ROCRED_UC_SET(x, v)			atomic_set(x->use_cnt, v)

extern struct cred init_cred;
extern struct task_security_struct init_sec;
extern struct kdp_usecnt init_cred_use_cnt;

extern void __init kdp_cred_init(void);
extern void __init kdp_do_early_param_setup(char *param, char *val);

// match for kernel/cred.c function
extern inline void set_cred_subscribers(struct cred *cred, int n);

// linux/cred.h
extern inline struct cred *get_new_cred(struct cred *cred);
extern inline void put_cred(const struct cred *_cred);
extern void put_rocred_rcu(struct rcu_head *rcu);
extern unsigned int kdp_get_usecount(struct cred *cred);

extern struct cred *prepare_ro_creds(struct cred *old, int kdp_cmd, u64 p);

extern int security_integrity_current(void);
extern void kdp_assign_pgd(struct task_struct *p);
extern inline int kdp_restrict_fork(struct filename *path);
extern void kdp_free_security(unsigned long tsec);

extern bool is_kdp_protect_addr(unsigned long addr);
#endif /* CONFIG_KDP_CRED */

#ifdef CONFIG_KDP_NS
/***************** KDP_NS *****************/
struct ns_param {
	u32 ns_buff_size;
	u32 ns_size;
	u32 bp_offset;
	u32 sb_offset;
	u32 flag_offset;
	u32 data_offset;
};

/* Populate all superblocks required for NS Protection */
enum __KDP_SB {
	KDP_SB_ROOTFS = 0,
	KDP_SB_ODM,
	KDP_SB_SYS,
	KDP_SB_VENDOR,
	KDP_SB_ART,
	KDP_SB_CRYPT,
	KDP_SB_DEX2OAT,
	KDP_SB_ADBD,
	KDP_SB_MAX
};

/* fs/pnode.h */
#define IS_MNT_SHARED(m) ((m)->mnt->mnt_flags & MNT_SHARED)
#define CLEAR_MNT_SHARED(m) kdp_clear_mnt_flags((m)->mnt,MNT_SHARED)
#define IS_MNT_UNBINDABLE(m) ((m)->mnt->mnt_flags & MNT_UNBINDABLE)
#define IS_MNT_MARKED(m) ((m)->mnt->mnt_flags & MNT_MARKED)
#define SET_MNT_MARK(m) kdp_set_mnt_flags((m)->mnt,MNT_MARKED)
#define CLEAR_MNT_MARK(m) kdp_clear_mnt_flags((m)->mnt,MNT_MARKED)
#define IS_MNT_LOCKED(m) ((m)->mnt->mnt_flags & MNT_LOCKED)

#define KDP_MOUNT_SYSTEM "/system"
#define KDP_MOUNT_SYSTEM_LEN strlen(KDP_MOUNT_SYSTEM)

#define KDP_MOUNT_PRODUCT "/product"
#define KDP_MOUNT_PRODUCT_LEN strlen(KDP_MOUNT_PRODUCT)

#define KDP_MOUNT_VENDOR "/vendor"
#define KDP_MOUNT_VENDOR_LEN strlen(KDP_MOUNT_VENDOR)

#define KDP_MOUNT_ART "/com.android.runtime"
#define KDP_MOUNT_ART_LEN strlen(KDP_MOUNT_ART)

#define KDP_MOUNT_CRYPT "/com.android.conscrypt"
#define KDP_MOUNT_CRYPT_LEN strlen(KDP_MOUNT_CRYPT)

#define KDP_MOUNT_DEX2OAT "/com.android.art"
#define KDP_MOUNT_DEX2OAT_LEN strlen(KDP_MOUNT_DEX2OAT)

#define KDP_MOUNT_ADBD "/com.android.adbd"
#define KDP_MOUNT_ADBD_LEN strlen(KDP_MOUNT_ADBD)

extern unsigned int ns_protect;

extern void __init kdp_mnt_init(void);
extern void __init kdp_init_mount_tree(struct vfsmount *mnt);

extern int kdp_mnt_alloc_vfsmount(struct mount *mnt);
extern void kdp_set_ns_data(struct vfsmount *mnt,void *data);
inline extern void kdp_set_mnt_root_sb(struct vfsmount *mnt, struct dentry *mnt_root, struct super_block *mnt_sb);
inline extern void kdp_set_mnt_flags(struct vfsmount *mnt, int flags);
inline extern void kdp_clear_mnt_flags(struct vfsmount *mnt,int flags);
inline extern void kdp_assign_mnt_flags(struct vfsmount *mnt, int flags);
extern int kdp_do_new_mount(struct vfsmount *mnt, struct path *path);

extern bool is_kdp_vfsmnt_cache(unsigned long addr);
extern void kdp_free_vfsmount(void *objp);

#endif /* CONFIG_KDP_NS */

#ifdef CONFIG_KDP_DMAP
static inline void dmap_prot(u64 addr,u64 order,u64 val)
{
	if(kdp_enable)
		uh_call(UH_APP_KDP, CHECK_DMAP, order, val, 0, 0);
}
#endif

#endif //__ASSEMBLY__
#endif //_RKP_H

