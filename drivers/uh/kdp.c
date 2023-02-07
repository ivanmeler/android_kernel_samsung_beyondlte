#include <asm-generic/sections.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/slub_def.h>

#include <linux/kdp.h>
#include <linux/cred.h>
#include <linux/security.h>
#include <linux/init_task.h>
#include <../../fs/mount.h>

#define VERITY_PARAM_LENGTH 20
#define KDP_CRED_SYS_ID 1000

/* security/selinux/include/objsec.h */
struct task_security_struct {
	u32 osid;               /* SID prior to last execve */
	u32 sid;                /* current SID */
	u32 exec_sid;           /* exec SID */
	u32 create_sid;         /* fscreate SID */
	u32 keycreate_sid;      /* keycreate SID */
	u32 sockcreate_sid;     /* fscreate SID */
	void *bp_cred;
};
/* security/selinux/hooks.c */
struct task_security_struct init_sec __kdp_ro;

bool kdp_enable __kdp_ro = false;
static int __check_verifiedboot __kdp_ro = 0;

static char verifiedbootstate[VERITY_PARAM_LENGTH];

#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
extern int ss_initialized __kdp_ro;
#endif

void __init kdp_init(void)
{
	struct kdp_init cred;
	memset((void *)&cred, 0, sizeof(kdp_init));

	cred.credSize		= sizeof(struct cred);
	cred.sp_size		= sizeof(struct task_security_struct);
	cred.pgd_mm			= offsetof(struct mm_struct, pgd);
	cred.uid_cred		= offsetof(struct cred, uid);
	cred.euid_cred		= offsetof(struct cred, euid);
	cred.gid_cred		= offsetof(struct cred, gid);
	cred.egid_cred		= offsetof(struct cred, egid);

	cred.bp_pgd_cred	= offsetof(struct cred, bp_pgd);
	cred.bp_task_cred	= offsetof(struct cred, bp_task);
	cred.type_cred		= offsetof(struct cred, type);
	cred.security_cred	= offsetof(struct cred, security);
	cred.usage_cred		= offsetof(struct cred, use_cnt);
	cred.cred_task		= offsetof(struct task_struct, cred);
	cred.mm_task		= offsetof(struct task_struct, mm);

	cred.pid_task		= offsetof(struct task_struct, pid);
	cred.rp_task		= offsetof(struct task_struct, real_parent);
	cred.comm_task		= offsetof(struct task_struct, comm);
	cred.bp_cred_secptr	= offsetof(struct task_security_struct, bp_cred);
	cred.verifiedbootstate	= (u64)verifiedbootstate;
	cred.selinux.empty 	= 0;
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
	cred.selinux.ss_initialized_va	= (u64)&ss_initialized;	
#else
	cred.selinux.ss_initialized_va	= 0;
#endif
	uh_call(UH_APP_KDP, KDP_INIT, (u64)&cred, 0, 0, 0);
}

static int __init verifiedboot_state_setup(char *str)
{
	strlcpy(verifiedbootstate, str, sizeof(verifiedbootstate));

	if (!strncmp(verifiedbootstate, "orange", sizeof("orange")))
		__check_verifiedboot = 1;
	return 0;
}
__setup("androidboot.verifiedbootstate=", verifiedboot_state_setup);

inline bool is_kdp_kmem_cache(struct kmem_cache *s)
{
	if (s->name &&
			(!strncmp(s->name, CRED_JAR_RO, strlen(CRED_JAR_RO)) ||
			 !strncmp(s->name, TSEC_JAR, strlen(TSEC_JAR)) ||
			 !strncmp(s->name, VFSMNT_JAR, strlen(VFSMNT_JAR))))
		return true;
	else
		return false;
}

#ifdef CONFIG_KDP_CRED
/*------------------------------------------------
 * CRED
 *------------------------------------------------
 */
struct kdp_usecnt init_cred_use_cnt = {
	.kdp_use_cnt = ATOMIC_INIT(4),
	.kdp_rcu_head.non_rcu = 0,
	.kdp_rcu_head.bp_cred = (void *)0,
	.kdp_rcu_head.reflected_cred = (void *)0,
};
static struct kmem_cache *cred_jar_ro;
static struct kmem_cache *tsec_jar;
static struct kmem_cache *usecnt_jar;

/* Dummy constructor to make sure we have separate slabs caches. */
static void cred_ctor(void *data) {}
static void sec_ctor(void *data) {}
static void usecnt_ctor(void *data) {}

void __init kdp_cred_init(void)
{
	if (kdp_enable) {
		cred_jar_ro = kmem_cache_create("cred_jar_ro", sizeof(struct cred),
				0, SLAB_HWCACHE_ALIGN|SLAB_PANIC|SLAB_ACCOUNT, cred_ctor);
		if (!cred_jar_ro)
			panic("Unable to create RO Cred cache\n");

		tsec_jar = kmem_cache_create("tsec_jar", sizeof(struct task_security_struct),
				0, SLAB_HWCACHE_ALIGN|SLAB_PANIC|SLAB_ACCOUNT, sec_ctor);
		if (!tsec_jar)
			panic("Unable to create RO security cache\n");

		usecnt_jar = kmem_cache_create("usecnt_jar", sizeof(struct kdp_usecnt),
				0, SLAB_HWCACHE_ALIGN|SLAB_PANIC|SLAB_ACCOUNT, usecnt_ctor);
		if (!usecnt_jar)
			panic("Unable to create use count jar\n");

		uh_call(UH_APP_KDP, CRED_INIT, (u64)cred_jar_ro->size, (u64)tsec_jar->size, 0, 0);
	}
}

unsigned int kdp_get_usecount(struct cred *cred)
{
	if (is_kdp_protect_addr((unsigned long )cred))
		return (unsigned int)ROCRED_UC_READ(cred);
	else
		return atomic_read(&cred->usage);
}

inline struct cred *get_new_cred(struct cred *cred)
{
	if (is_kdp_protect_addr((unsigned long)cred))
		ROCRED_UC_INC(cred);
	else
		atomic_inc(&cred->usage);
	return cred;
}

/* copy logic - include/linux/cred.h */
inline void put_cred(const struct cred *_cred)
{
	struct cred *cred = (struct cred *) _cred;

	validate_creds(cred);

	if (is_kdp_protect_addr((unsigned long)cred)) {
		if (ROCRED_UC_DEC_AND_TEST(cred))
			__put_cred(cred);
	} else {
		if (atomic_dec_and_test(&(cred)->usage))
			__put_cred(cred);
	}
}

/* match for kernel/cred.c function */
inline void set_cred_subscribers(struct cred *cred, int n)
{
#ifdef CONFIG_DEBUG_CREDENTIALS
	atomic_set(&cred->subscribers, n);
#endif
}

/* Check whether the address belong to Cred Area */
bool is_kdp_protect_addr(unsigned long addr)
{
	struct kmem_cache *s;
	struct page *page;
	void *objp = (void *)addr;

	if (!objp)
		return false;

	if (!kdp_enable)
		return false;

	if ((addr == ((unsigned long)&init_cred)) ||
			(addr == ((unsigned long)&init_sec)))
		return true;

	page = virt_to_head_page(objp);
	s = page->slab_cache;
	if (s && (s == cred_jar_ro || s == tsec_jar))
		return true;

	return false;
}

/* We use another function to free protected creds. */
void put_rocred_rcu(struct rcu_head *rcu)
{
	struct cred *cred = container_of(rcu, struct ro_rcu_head, rcu)->bp_cred;

	if (ROCRED_UC_READ(cred) != 0)
		panic("RO_CRED: put_rocred_rcu() sees %p with usage %d\n",
				cred, ROCRED_UC_READ(cred));

	security_cred_free(cred);
	key_put(cred->session_keyring);
	key_put(cred->process_keyring);
	key_put(cred->thread_keyring);
	key_put(cred->request_key_auth);
	if (cred->group_info)
		put_group_info(cred->group_info);
	free_uid(cred->user);
	put_user_ns(cred->user_ns);
	if(cred->use_cnt)
		kmem_cache_free(usecnt_jar,(void *)cred->use_cnt);
	kmem_cache_free(cred_jar_ro, cred);
}

/* prepare_ro_creds - Prepare a new set of credentials which is protected by KDP */
struct cred *prepare_ro_creds(struct cred *old, int kdp_cmd, u64 p)
{
	u64 pgd = (u64)(current->mm ? current->mm->pgd : swapper_pg_dir);
	struct cred *new_ro = NULL;
	struct cred_param param_data;
	void *use_cnt_ptr = NULL;
	void *rcu_ptr = NULL;
	void *tsec = NULL;

	new_ro = kmem_cache_alloc(cred_jar_ro, GFP_KERNEL);
	if (!new_ro)
		panic("[%d] : kmem_cache_alloc() failed", kdp_cmd);

	use_cnt_ptr = kmem_cache_alloc(usecnt_jar, GFP_KERNEL);
	if (!use_cnt_ptr)
		panic("[%d] : Unable to allocate usage pointer\n", kdp_cmd);

	// get_usecnt_rcu
	rcu_ptr = (struct ro_rcu_head *)((atomic_t *)use_cnt_ptr + 1);
	((struct ro_rcu_head *)rcu_ptr)->bp_cred = (void *)new_ro;

	tsec = kmem_cache_alloc(tsec_jar, GFP_KERNEL);
	if (!tsec)
		panic("[%d] : Unable to allocate security pointer\n", kdp_cmd);

	// init
	memset((void *)&param_data, 0, sizeof(struct cred_param));
	param_data.cred = old;
	param_data.cred_ro = new_ro;
	param_data.use_cnt_ptr = use_cnt_ptr;
	param_data.sec_ptr = tsec;
	param_data.type = kdp_cmd;
	param_data.use_cnt = (u64)p;

	uh_call(UH_APP_KDP, PREPARE_RO_CRED, (u64)&param_data, (u64)current, 0, 0);
	if (kdp_cmd == CMD_COPY_CREDS) {
		if ((new_ro->bp_task != (void *)p) ||
				new_ro->security != tsec ||
				new_ro->use_cnt != use_cnt_ptr) {
			panic("[%d]: KDP Call failed task=0x%lx:0x%lx, sec=0x%lx:0x%lx, usecnt=0x%lx:0x%lx",
					kdp_cmd, new_ro->bp_task, (void *)p,
					new_ro->security, tsec, new_ro->use_cnt, use_cnt_ptr);
		}
	} else {
		if ((new_ro->bp_task != current) ||
				(current->mm && new_ro->bp_pgd != (void *)pgd) ||
				(new_ro->security != tsec) ||
				(new_ro->use_cnt != use_cnt_ptr)) {
			panic("[%d]: KDP Call failed task=0x%lx:0x%lx, sec=0x%lx:0x%lx, usecnt=0x%lx:0x%lx, pgd=0x%lx:0x%lx",
					kdp_cmd, new_ro->bp_task, current, new_ro->security, tsec,
					new_ro->use_cnt, use_cnt_ptr, new_ro->bp_pgd, (void *)pgd);
		}
	}

	GET_ROCRED_RCU(new_ro)->non_rcu = old->non_rcu;
	GET_ROCRED_RCU(new_ro)->reflected_cred = 0;
	ROCRED_UC_SET(new_ro, 2);

	set_cred_subscribers(new_ro, 0);
	get_group_info(new_ro->group_info);
	get_uid(new_ro->user);
	get_user_ns(new_ro->user_ns);

#ifdef CONFIG_KEYS
	key_get(new_ro->session_keyring);
	key_get(new_ro->process_keyring);
	key_get(new_ro->thread_keyring);
	key_get(new_ro->request_key_auth);
#endif

	validate_creds(new_ro);
	return new_ro;
}

/* security/selinux/hooks.c */
static bool is_kdp_tsec_jar(unsigned long addr)
{
	struct kmem_cache *s;
	struct page *page;
	void *objp = (void *)addr;

	if (!objp)
		return false;

	page = virt_to_head_page(objp);
	s = page->slab_cache;
	if (s && s == tsec_jar)
		return true;
	return false;
}

static inline int chk_invalid_kern_ptr(u64 tsec)
{
	return (((u64)tsec >> 36) != (u64)0xFFFFFFC);
}

void kdp_free_security(unsigned long tsec)
{
	if (!tsec || chk_invalid_kern_ptr(tsec))
		return;

	if (is_kdp_tsec_jar(tsec))
		kmem_cache_free(tsec_jar, (void *)tsec);
	else
		kfree((void *)tsec);
}

void kdp_assign_pgd(struct task_struct *p)
{
	u64 pgd = (u64)(p->mm ? p->mm->pgd : swapper_pg_dir);

	uh_call(UH_APP_KDP, SET_CRED_PGD, (u64)p->cred, (u64)pgd, 0, 0);
}

struct task_security_struct init_sec __kdp_ro;
static inline unsigned int cmp_sec_integrity(const struct cred *cred, struct mm_struct *mm)
{
	if (cred->bp_task != current)
		printk(KERN_ERR "[KDP] cred->bp_task: 0x%lx, current: 0x%lx\n",
				cred->bp_task, current);

	if (mm && (cred->bp_pgd != swapper_pg_dir) && (cred->bp_pgd != mm->pgd ))
		printk(KERN_ERR "[KDP] mm: 0x%lx, cred->bp_pgd: 0x%lx, swapper_pg_dir: %p, mm->pgd: 0x%lx\n",
				mm, cred->bp_pgd, swapper_pg_dir, mm->pgd, cred->bp_pgd);

	return ((cred->bp_task != current) ||
			(mm && (!( in_interrupt() || in_softirq())) &&
			 (cred->bp_pgd != swapper_pg_dir) &&
			 (cred->bp_pgd != mm->pgd)));
}

static inline bool is_kdp_invalid_cred_sp(u64 cred, u64 sec_ptr)
{
	struct task_security_struct *tsec = (struct task_security_struct *)sec_ptr;
	u64 cred_size = sizeof(struct cred);
	u64 tsec_size = sizeof(struct task_security_struct);

	if ((cred == (u64)&init_cred) && (sec_ptr == (u64)&init_sec))
		return false;

	if (!is_kdp_protect_addr(cred) ||
			!is_kdp_protect_addr(cred + cred_size) ||
			!is_kdp_protect_addr(sec_ptr) ||
			!is_kdp_protect_addr(sec_ptr + tsec_size)) {
		printk(KERN_ERR, "[KDP] cred: %d, cred + sizeof(cred): %d, sp: %d, sp + sizeof(tsec): %d",
				is_kdp_protect_addr(cred),
				is_kdp_protect_addr(cred + cred_size),
				is_kdp_protect_addr(sec_ptr),
				is_kdp_protect_addr(sec_ptr + tsec_size));
		return true;
	}

	if ((u64)tsec->bp_cred != cred) {
		printk(KERN_ERR, "[KDP] %s: tesc->bp_cred: %lx, cred: %lx\n",
				__func__, (u64)tsec->bp_cred, cred);
		return true;
	}

	return false;
}

inline int kdp_restrict_fork(struct filename *path)
{
	struct cred *shellcred;

	if (!strcmp(path->name, "/system/bin/patchoat") ||
			!strcmp(path->name, "/system/bin/idmap2")) {
		return 0;
	}

	if(KDP_IS_NONROOT(current)) {
		shellcred = prepare_creds();
		if (!shellcred)
			return 1;

		shellcred->uid.val = 2000;
		shellcred->gid.val = 2000;
		shellcred->euid.val = 2000;
		shellcred->egid.val = 2000;

		commit_creds(shellcred);
	}
	return 0;
}

/* This function is related Namespace */
#ifdef CONFIG_KDP_NS
static unsigned int cmp_ns_integrity(void)
{
	struct mount *root = NULL;
	struct nsproxy *nsp = NULL;

	if (in_interrupt() || in_softirq())
		return 0;

	nsp = current->nsproxy;
	if (!ns_protect || !nsp || !nsp->mnt_ns)
		return 0;

	root = current->nsproxy->mnt_ns->root;
	if (root != root->mnt->bp_mount) {
		printk(KERN_ERR "[KDP] NameSpace Mismatch %lx != %lx\n nsp: 0x%lx, mnt_ns: 0x%lx\n",
				root, root->mnt->bp_mount, nsp, nsp->mnt_ns);
		return 1;
	}

	return 0;
}
#endif // end CONFIG_KDP_NS

/* Main function to verify cred security context of a process */
int security_integrity_current(void)
{
	const struct cred *cur_cred = current_cred();
	rcu_read_lock();
	if (kdp_enable &&
			(is_kdp_invalid_cred_sp((u64)cur_cred, (u64)cur_cred->security)
			 || cmp_sec_integrity(cur_cred, current->mm) 
#ifdef CONFIG_KDP_NS
			 || cmp_ns_integrity())) {
#else
		)) {
#endif
			rcu_read_unlock();
			panic("KDP CRED PROTECTION VIOLATION\n");
		}
		rcu_read_unlock();
		return 0;
	}
#endif

#ifdef CONFIG_KDP_NS
/*------------------------------------------------
 * Namespace
 *------------------------------------------------
 */
unsigned int ns_protect __kdp_ro = 0;
static int dex2oat_count = 0;
static DEFINE_SPINLOCK(mnt_vfsmnt_lock);

static struct super_block *rootfs_sb __kdp_ro = NULL;
static struct super_block *sys_sb __kdp_ro = NULL;
static struct super_block *odm_sb __kdp_ro = NULL;
static struct super_block *vendor_sb __kdp_ro = NULL;
static struct super_block *art_sb __kdp_ro = NULL;
static struct super_block *crypt_sb	__kdp_ro = NULL;
static struct super_block *dex2oat_sb	__kdp_ro = NULL;
static struct super_block *adbd_sb		__kdp_ro = NULL;
static struct kmem_cache *vfsmnt_cache __read_mostly;

extern int __is_kdp_recovery;

void cred_ctor_vfsmount(void *data)
{
	/* Dummy constructor to make sure we have separate slabs caches. */
}
void __init kdp_mnt_init(void)
{
	struct ns_param nsparam;

	vfsmnt_cache = kmem_cache_create("vfsmnt_cache", sizeof(struct vfsmount),
			0, SLAB_HWCACHE_ALIGN | SLAB_PANIC, cred_ctor_vfsmount);

	if (!vfsmnt_cache)
		panic("Failed to allocate vfsmnt_cache\n");

	memset((void *)&nsparam, 0, sizeof(struct ns_param));
	nsparam.ns_buff_size = (u64)vfsmnt_cache->size;
	nsparam.ns_size = (u64)sizeof(struct vfsmount);
	nsparam.bp_offset = (u64)offsetof(struct vfsmount, bp_mount);
	nsparam.sb_offset = (u64)offsetof(struct vfsmount, mnt_sb);
	nsparam.flag_offset = (u64)offsetof(struct vfsmount,mnt_flags);
	nsparam.data_offset = (u64)offsetof(struct vfsmount, data);

	uh_call(UH_APP_KDP, NS_INIT, (u64)&nsparam, 0, 0, 0);
}

void __init kdp_init_mount_tree(struct vfsmount *mnt)
{
	if (!rootfs_sb)
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&rootfs_sb, (u64)mnt, KDP_SB_ROOTFS, 0);
}

bool is_kdp_vfsmnt_cache(unsigned long addr)
{
	static void *objp;
	static struct kmem_cache *s;
	static struct page *page;

	objp = (void *)addr;

	if (!objp)
		return false;

	page = virt_to_head_page(objp);
	s = page->slab_cache;
	if (s && s == vfsmnt_cache)
		return true;
	return false;
}

inline void kdp_set_mnt_root_sb(struct vfsmount *mnt, struct dentry *mnt_root, struct super_block *mnt_sb)
{
	uh_call(UH_APP_KDP, SET_NS_ROOT_SB, (u64)mnt, (u64)mnt_root, (u64)mnt_sb, 0);
}

inline void kdp_assign_mnt_flags(struct vfsmount *mnt, int flags)
{
	uh_call(UH_APP_KDP, SET_NS_FLAGS, (u64)mnt, (u64)flags, 0, 0);
}

inline void kdp_clear_mnt_flags(struct vfsmount *mnt, int flags)
{
	int f = mnt->mnt_flags;

	f &= ~flags;
	kdp_assign_mnt_flags(mnt, f);
}

void kdp_set_mnt_flags(struct vfsmount *mnt, int flags)
{
	int f = mnt->mnt_flags;

	f |= flags;
	kdp_assign_mnt_flags(mnt, f);
}

void kdp_set_ns_data(struct vfsmount *mnt, void *data)
{
	uh_call(UH_APP_KDP, SET_NS_DATA, (u64)mnt, (u64)data, 0, 0);
}

int kdp_mnt_alloc_vfsmount(struct mount *mnt)
{
	struct vfsmount *vfsmnt = NULL;

	vfsmnt = kmem_cache_alloc(vfsmnt_cache, GFP_KERNEL);
	if (!vfsmnt)
		return 1;

	spin_lock(&mnt_vfsmnt_lock);
	uh_call(UH_APP_KDP, ALLOC_VFSMOUNT, (u64)vfsmnt, (u64)mnt, 0, 0);
	mnt->mnt = vfsmnt;
	spin_unlock(&mnt_vfsmnt_lock);

	return 0;
}

void kdp_free_vfsmount(void *objp)
{
	kmem_cache_free(vfsmnt_cache, objp);
}

static void kdp_populate_sb(char *mount_point, struct vfsmount *mnt)
{
	if (!mount_point || !mnt)
		return;

	if (!odm_sb && !strncmp(mount_point, KDP_MOUNT_PRODUCT, KDP_MOUNT_PRODUCT_LEN))
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&odm_sb, (u64)mnt, KDP_SB_ODM, 0);
	else if (!sys_sb && !strncmp(mount_point, KDP_MOUNT_SYSTEM, KDP_MOUNT_SYSTEM_LEN))
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&sys_sb, (u64)mnt, KDP_SB_SYS, 0);
	else if (!vendor_sb && !strncmp(mount_point, KDP_MOUNT_VENDOR, KDP_MOUNT_VENDOR_LEN))
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&vendor_sb, (u64)mnt, KDP_SB_VENDOR, 0);
	else if (!art_sb && !strncmp(mount_point, KDP_MOUNT_ART, KDP_MOUNT_ART_LEN - 1))
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&art_sb, (u64)mnt, KDP_SB_ART, 0);
	else if (!crypt_sb && strstr(mount_point, KDP_MOUNT_CRYPT))
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&crypt_sb, (u64)mnt, KDP_SB_CRYPT, 0);
	else if (!dex2oat_sb && !strncmp(mount_point, KDP_MOUNT_DEX2OAT, KDP_MOUNT_DEX2OAT_LEN - 1))
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&dex2oat_sb, (u64)mnt, KDP_SB_DEX2OAT, 0);
	else if (!dex2oat_count && !strncmp(mount_point, KDP_MOUNT_DEX2OAT, KDP_MOUNT_DEX2OAT_LEN)) {
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&dex2oat_sb, (u64)mnt, KDP_SB_DEX2OAT, 0);
		dex2oat_count++;
	}
	else if (!adbd_sb && !strncmp(mount_point, KDP_MOUNT_ADBD, KDP_MOUNT_ADBD_LEN - 1))
		uh_call(UH_APP_KDP, SET_NS_SB_VFSMOUNT, (u64)&adbd_sb, (u64)mnt, KDP_SB_ADBD, 0);
}

int kdp_do_new_mount(struct vfsmount *mnt, struct path *path)
{
	char *buf = NULL;
	char *dir_name;

	buf = kzalloc(PATH_MAX, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	dir_name = dentry_path_raw(path->dentry, buf, PATH_MAX);
	if (!sys_sb || !odm_sb || !vendor_sb || !art_sb || !crypt_sb || !dex2oat_sb || !dex2oat_count || !adbd_sb)
		kdp_populate_sb(dir_name, mnt);

	kfree(buf);

	return 0;
}
#endif
